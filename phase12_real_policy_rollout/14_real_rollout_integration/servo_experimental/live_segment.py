"""Command-referenced, bounded ServoL candidates. No ROS calls in this module.

Quintic time scaling replaces velocity integration. Mid-segment retargets are
latest-only queued, not applied by resetting a moving segment to zero speed.
The existing segment reaches a zero-velocity/acceleration boundary first.
This deliberately trades target-update latency for a bounded C2 path; it is not
an unbounded curve blending method and never changes source timestamps.
Hardware release evidence and following tolerances must be configured explicitly.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from servo_experimental.bounded_segment import CommandSegmentInterpolator
from servo_experimental.reference_replay import pose_distance


def stream_planning_limits(base, config):
    """Isolate ServoL stream profile from shared policy/MoveLine thresholds."""
    effective = dict(base)
    for key in ('translation_velocity_mm_s', 'translation_acceleration_mm_s2',
                'rotation_velocity_deg_s', 'rotation_acceleration_deg_s2'):
        value = config['stream_planning'][key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid_stream_planning_limit:'+key)
        effective[key] = value
    return effective


def release_blockers(config):
    required = ('normalized_euler_controller_verified', 'physical_stop_verified',
                'controller_joint_protection_verified')
    missing = [key for key in required if config.get(key) is not True]
    for key in ('following_translation_mm', 'following_rotation_deg', 'feedback_max_age_s'):
        value = config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            missing.append(key)
    return missing


def require_release(config):
    missing = release_blockers(config)
    if missing:
        raise PermissionError('servo_release_blocked:' + ','.join(missing))


class LiveCommandSegment(CommandSegmentInterpolator):
    period = .05
    max_tick_gap_s = .1  # Same existing stream tick watchdog, not a new allowance.

    def __init__(self, pose, limits, *, transport, protection):
        super().__init__(pose, limits)
        require_release(protection)
        self.transport = transport
        self.protection = protection
        self.velocity = np.zeros(6)
        self.history = []
        self.initial = self.pose()
        self.pending = None
        self.replaced_pending = 0
        self.activated_segments = 0
        self.velocity_dt = self.period
        self.braking_plans = 0
        self.last_input_time = None
        self.input_period = 1/self.limits['control_rate_hz']

    def pose(self):
        return self.last.tolist()

    def set_action(self, action, source, now):
        if self.closed:
            raise RuntimeError('segment_inhibited')
        action = np.asarray(action, dtype=float)
        if action.shape != (7,) or not np.all(np.isfinite(action)):
            self.closed = True
            raise ValueError('invalid_limited_action')
        if (not math.isfinite(now) or not math.isfinite(source) or
                not 0 <= now-source <= self.limits['max_action_age_sec']):
            self.closed = True
            raise ValueError('stale_action')
        latest_source = (self.pending[1] if self.pending is not None else
                         self.segment['source'] if self.segment is not None else None)
        if latest_source is not None and source <= latest_source:
            self.closed = True
            raise ValueError('action_source_regression_or_duplicate')
        if self.last_tick is not None and now < self.last_tick:
            self.closed = True
            raise ValueError('segment_clock_regression')
        if self.last_input_time is not None:
            if now<=self.last_input_time:
                self.closed = True
                raise ValueError('action_input_clock_regression')
            self.input_period = now-self.last_input_time
        self.last_input_time = now
        if self.segment is not None and not self._at_rest():
            if self.pending is not None:
                self.replaced_pending += 1
            self.pending = (action.copy(), source)
            self._brake_within_segment(now)
            return dict(state='QUEUED_LATEST_UNTIL_REST', source_monotonic=source)
        if self.pending is not None:
            self.replaced_pending += 1
            self.pending = None
        self._activate(action, source, now)
        return dict(state='ACTIVATED_AT_REST', source_monotonic=source)

    def _at_rest(self):
        # A generated endpoint has analytic velocity AND acceleration zero.
        # Keep the previous interval-average velocity for the independent
        # finite-difference check; resetting that history would hide a jump.
        # The new profile's first sample holds this endpoint (u=0).
        return self.segment.get('profile_done', False)

    @staticmethod
    def _range(coefficients):
        """Exact polynomial extrema on [0,1], not a sampled safety check."""
        coefficients = np.polynomial.polynomial.polytrim(coefficients, tol=1e-14)
        derivative = np.polynomial.polynomial.polyder(coefficients)
        roots = np.polynomial.polynomial.polyroots(derivative)
        points = [0., 1.] + [float(r.real) for r in roots
                              if abs(r.imag)<1e-8 and 0<r.real<1]
        values = np.polynomial.polynomial.polyval(points, coefficients)
        return float(min(values)), float(max(values))

    def _brake_within_segment(self, now):
        """C2 stop along the original straight/SO(3) arc, never past its end.

        Initial progress/velocity/acceleration are inherited at the last command
        sample. Search only profiles with analytically bounded derivatives.
        If no shorter stop is feasible, retain the already bounded old profile;
        source-age expiry still inhibits, never rebases or relaxes a threshold.
        """
        s = self.segment
        if self.last_tick is None or s.get('braking') or s.get('profile_done'):
            return
        elapsed = self.last_tick-s['activated']
        u = float(np.clip(elapsed/s['duration'], 0, 1))
        coefficients = s['profile_coeff']
        q = float(np.polynomial.polynomial.polyval(u, coefficients))
        v = float(np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(coefficients))/s['duration'])
        a = float(np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(coefficients,2))/s['duration']**2)
        minimum = max(self.period, now-self.last_tick)
        remaining = s['duration']-elapsed
        if remaining<=minimum or v<1e-12:
            return
        d, r = s['translation_budget'], s['rotation_budget']
        vmax = min([bound/budget for bound,budget in (
            (self.limits['translation_velocity_mm_s'],d),
            (self.limits['rotation_velocity_deg_s'],r)) if budget>1e-12] or [math.inf])
        amax = min([bound/budget for bound,budget in (
            (self.limits['translation_acceleration_mm_s2'],d),
            (self.limits['rotation_acceleration_deg_s2'],r)) if budget>1e-12] or [math.inf])
        grid = np.linspace(0.,1.,33)
        powers = np.array([grid**i for i in range(6)])
        for duration in np.linspace(minimum,remaining,40):
            endpoints = sorted(set([1.] + [q+v*duration*f+a*duration**2*g
                for f in (.25,.35,.5,.65,.8) for g in (0.,.05,.1)]))
            endpoints = np.array([end for end in endpoints if q<end<=1.])
            if not len(endpoints):
                continue
            delta=endpoints-q;vd=v*duration;ad=a*duration**2
            candidates=np.zeros((len(endpoints),6));candidates[:,:3]=[q,vd,.5*ad]
            candidates[:,3]=10*delta-6*vd-1.5*ad
            candidates[:,4]=-15*delta+8*vd+1.5*ad
            candidates[:,5]=6*delta-3*vd-.5*ad
            # Vectorized rejection prefilter ONLY. Every accepted candidate is
            # still proved against exact polynomial extrema below; grid values
            # are never used as sufficient evidence of safety.
            sampled_q=candidates@powers
            sampled_v=(candidates[:,1:]*np.arange(1,6)/duration)@powers[:5]
            sampled_a=(candidates[:,2:]*np.arange(2,6)*np.arange(1,5)/duration**2)@powers[:4]
            possible=((sampled_q.min(axis=1)>=q-1e-9)&(sampled_q.max(axis=1)<=1+1e-9)&
                (sampled_v.min(axis=1)>=-1e-9)&(sampled_v.max(axis=1)<=vmax+1e-9)&
                (np.abs(sampled_a).max(axis=1)<=amax+1e-9))
            for end,c in zip(endpoints[possible],candidates[possible]):
                qlo,qhi = self._range(c)
                vlo,vhi = self._range(np.polynomial.polynomial.polyder(c)/duration)
                alo,ahi = self._range(np.polynomial.polynomial.polyder(c,2)/duration**2)
                if (qlo<q-1e-9 or qhi>1+1e-9 or vlo<-1e-9 or
                        vhi>vmax+1e-9 or max(abs(alo),abs(ahi))>amax+1e-9):
                    continue
                s.update(profile_coeff=c,activated=self.last_tick,duration=float(duration),
                    braking=True,brake_endpoint_fraction=float(end),
                    brake_initial_progress=q,brake_initial_velocity=v,brake_initial_acceleration=a)
                self.braking_plans += 1
                return

    def _activate(self, action, source, now):
        distance = np.linalg.norm(action[:3])*1000
        angle = math.degrees(Rotation.from_rotvec(action[3:6]).magnitude())
        # max dq/du=1.875 and max |d2q/du2|=10/sqrt(3).
        # Estimate inference spacing from arrivals, not from queued activations.
        # Queueing delay must not recursively lengthen every subsequent segment.
        period = self.input_period
        duration = max(period, 1.875*distance/self.limits['translation_velocity_mm_s'],
            1.875*angle/self.limits['rotation_velocity_deg_s'],
            math.sqrt((10/math.sqrt(3))*distance/self.limits['translation_acceleration_mm_s2']),
            math.sqrt((10/math.sqrt(3))*angle/self.limits['rotation_acceleration_deg_s2']))
        # Finish the bounded motion before the ORIGINAL action expires. Reserve
        # one existing maximum tick interval for endpoint sampling and one for
        # activating the next queued goal. Never renew source time or age limit.
        horizon = source+self.limits['max_action_age_sec']-now-2*self.max_tick_gap_s
        if not math.isfinite(horizon) or horizon<self.period:
            self.closed = True
            raise ValueError('insufficient_fresh_action_horizon')
        duration = min(duration,horizon)
        progress = 1.
        for budget,speed,acceleration in (
            (distance,self.limits['translation_velocity_mm_s'],self.limits['translation_acceleration_mm_s2']),
            (angle,self.limits['rotation_velocity_deg_s'],self.limits['rotation_acceleration_deg_s2'])):
            if budget>1e-12:
                progress=min(progress,speed*duration/(1.875*budget),
                             acceleration*duration**2/((10/math.sqrt(3))*budget))
        # This limits executed progress using the existing dynamic bounds; it
        # does not rewrite the raw/limited action, original target, or budget.
        # Partial target execution is explicit in every candidate's provenance.
        super().set_action(action, source, now, period=duration)
        self.segment.update(profile_coeff=np.array([0.,0.,0.,10.,-15.,6.])*progress,
                            fresh_horizon_s=horizon, planned_execution_fraction=progress,
                            profile_done=False, braking=False)
        self.activated_segments += 1

    def verify_feedback(self, feedback, now):
        received = feedback.get('received')
        if received is None or not 0 <= now-received < self.protection['feedback_max_age_s']:
            self.closed = True
            raise RuntimeError('stale_controller_feedback')
        reference = self.initial
        for stamp, command in reversed(self.history):
            if stamp <= received:
                reference = command
                break
        t, r = pose_distance(reference, feedback['pose'])
        if (t > self.protection['following_translation_mm'] or
                r > self.protection['following_rotation_deg']):
            self.closed = True
            raise RuntimeError('following_error_limit')
        return dict(translation_mm=t, rotation_deg=r, received=received)

    def mark_published(self, stamp, pose):
        self.history.append((stamp, list(pose)))
        # Retain enough history for the configured feedback age, plus anchor.
        while len(self.history)>2 and self.history[1][0]<stamp-self.protection['feedback_max_age_s']-1:
            self.history.pop(0)

    def tick(self, now):
        if self.closed:
            raise RuntimeError('segment_inhibited')
        if self.pending is not None and self.segment is not None and self._at_rest():
            action, source = self.pending
            self._activate(action, source, now)
            self.pending = None
        if self.segment is None:
            return None
        previous = self.last.copy()
        previous_tick = self.last_tick
        s = self.segment
        if not math.isfinite(now) or now < s['activated']:
            self.closed = True
            raise ValueError('segment_clock_regression')
        u = float(np.clip((now-s['activated'])/s['duration'], 0, 1))
        fraction = float(np.polynomial.polynomial.polyval(u,s['profile_coeff']))
        fraction = float(np.clip(fraction,0.,1.))
        # Base interpolator supplies clock, age, workspace and cumulative budget
        # checks. Warp its activation only for interpolation; keep age unchanged.
        activated = s['activated']
        s['activated'] = now-fraction*s['duration']
        try:
            row = super().tick(now)
        finally:
            s['activated'] = activated
        dt = self.period if previous_tick is None else now-previous_tick
        r0 = Rotation.from_euler('ZYZ', previous[3:], degrees=True)
        r1 = Rotation.from_euler('ZYZ', self.last[3:], degrees=True)
        velocity = np.r_[(self.last[:3]-previous[:3])/dt,
                          (r1*r0.inv()).as_rotvec()*180/math.pi/dt]
        # Both velocities are averages over their own (possibly unequal) sample
        # intervals. Their centers are separated by half the interval sum.
        acceleration_dt = .5*(self.velocity_dt+dt)
        acceleration = (velocity-self.velocity)/acceleration_dt
        checks = ((np.linalg.norm(velocity[:3]), self.limits['translation_velocity_mm_s']),
                  (np.linalg.norm(velocity[3:]), self.limits['rotation_velocity_deg_s']),
                  (np.linalg.norm(acceleration[:3]), self.limits['translation_acceleration_mm_s2']),
                  (np.linalg.norm(acceleration[3:]), self.limits['rotation_acceleration_deg_s2']))
        if any(value>limit+1e-6 for value,limit in checks):
            self.closed = True
            raise RuntimeError('stream_dynamic_limit')
        self.velocity = velocity
        self.velocity_dt = dt
        s['profile_done'] = u>=1.
        row.update(vel=list(self.transport['velocity_mm_s_deg_s']),
            acc=list(self.transport['acceleration_mm_s2_deg_s2']), time=self.transport['target_time_s'],
            generator='COMMAND_REFERENCE_C2_BRAKE_SLERP_NORMALIZED_ZYZ',
            segment_start_pose=s['start'].tolist(), segment_target_pose=s['target'].tolist(),
            planned_execution_fraction=s['planned_execution_fraction'],
            fresh_horizon_s=s['fresh_horizon_s'],
            linear_time_fraction=u, smooth_time_fraction=fraction,
            retarget_policy='LATEST_ONLY_AFTER_C2_REST_BOUNDARY',
            pending_source_monotonic=None if self.pending is None else self.pending[1],
            replaced_pending=self.replaced_pending, activated_segments=self.activated_segments,
            braking=s['braking'], braking_plans=self.braking_plans,
            brake_endpoint_fraction=s.get('brake_endpoint_fraction'),
            acceleration_comparison_dt_s=acceleration_dt,
            actual_tick_dt_s=dt, planning_velocity=velocity.tolist(),
            planning_acceleration=acceleration.tolist())
        return row
