"""Geometric sequence completion does not certify object grasp."""
import math

class TaskPhase:
    def __init__(self,config):self.config=config;self.phase='APPROACH';self.started=None
    def update(self,tcp,*,command_closed=False,safety_ok=True,elapsed=0):
        if not safety_ok or elapsed>self.config['max_duration_s']:self.phase='ABORT';return self.phase
        goal=self.config['grasp_pose_m'];xy=math.dist(tcp[:2],goal[:2]);z=tcp[2]
        if self.phase=='APPROACH' and xy<=self.config['grasp_xy_tolerance_m'] and abs(z-self.config['pregrasp_z_m'])<=self.config['pregrasp_z_tolerance_m']:self.phase='DESCEND'
        elif self.phase=='DESCEND' and xy<=self.config['grasp_xy_tolerance_m'] and abs(z-goal[2])<=self.config['grasp_z_tolerance_m']:self.phase='GRASP_CLOSE'
        elif self.phase=='GRASP_CLOSE' and command_closed:self.phase='LIFT'
        elif self.phase=='LIFT' and command_closed and z>=goal[2]+self.config['lift_height_threshold_m']:self.phase='COMPLETE'
        return self.phase
    def report(self):return {'phase':self.phase,'task_sequence_complete':self.phase=='COMPLETE','physical_grasp_success':'UNVERIFIED'}
