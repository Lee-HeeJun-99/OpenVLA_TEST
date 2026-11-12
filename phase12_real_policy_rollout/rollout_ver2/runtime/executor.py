from __future__ import annotations

MODES=("dryrun_offline","dryrun_live","gripper_test","stop_test_movej",
       "stop_test_servoj","synthetic","policy")

MODE_REQUIRED_SETTINGS={
  "dryrun_offline":(),
  "dryrun_live":("robot_state_topic_unverified","joint_states_topic_unverified",
                 "camera_topic_unverified","feedback_stale_s_candidate"),
  "gripper_test":("tool_digital_output_service","gripper.open_output_index",
                  "gripper.close_output_index","zero_velocity_rad_s_candidate"),
  "stop_test_movej":("stop_service","move_stop_mode_candidate_unverified",
                     "move_joint_service","stop_test_movej.target_joint_deg",
                     "stop_test_movej.velocity_deg_s_candidate",
                     "stop_test_movej.acceleration_deg_s2_unverified",
                     "zero_velocity_rad_s_candidate","zero_velocity_hold_s_candidate"),
  "stop_test_servoj":("stop_service","move_stop_mode_candidate_unverified",
                      "servoj_time_unverified","servoj_velocity_unverified",
                      "servoj_acceleration_unverified","tracking_position_m_candidate",
                      "tracking_rotation_deg_candidate","zero_velocity_rad_s_candidate",
                      "zero_velocity_hold_s_candidate"),
  "synthetic":("stop_service","move_stop_mode_candidate_unverified",
               "servoj_time_unverified","servoj_velocity_unverified",
               "servoj_acceleration_unverified","tracking_position_m_candidate",
               "tracking_rotation_deg_candidate"),
  "policy":("stop_service","move_stop_mode_candidate_unverified",
            "servoj_time_unverified","servoj_velocity_unverified",
            "servoj_acceleration_unverified","tracking_position_m_candidate",
            "tracking_rotation_deg_candidate"),
}


class V2Executor:
    """One mode dispatcher. It never creates ROS entities by itself."""
    def __init__(self, gate, *, physical_enabled=False, joint_limits=None,
                 runtime_config=None):
        self.gate=gate; self.physical_enabled=bool(physical_enabled)
        if runtime_config is None:
            from runtime.config import load_runtime_config
            runtime_config=load_runtime_config()
        self.runtime_config=runtime_config
        if joint_limits is None:
            from controller.joint_limits import load_joint_limits
            joint_limits=load_joint_limits()
        self.joint_limits=joint_limits

    def authorize(self, mode, human_confirmation):
        if mode not in MODES: raise ValueError("unknown mode")
        if human_confirmation != "I_CONFIRM_V2_MODE:"+mode:
            raise PermissionError("FRESH_HUMAN_CONFIRMATION_REQUIRED")
        self.gate.require(mode)
        self.joint_limits.require_extension_gates(self.gate)
        missing=[]
        for path in MODE_REQUIRED_SETTINGS[mode]:
            value=self.runtime_config
            for key in path.split("."):
                value=value.get(key) if isinstance(value,dict) else None
            if value is None: missing.append(path)
        if missing:
            raise PermissionError("V2_SETTINGS_UNVERIFIED:"+",".join(missing))
        if mode not in ("dryrun_offline","dryrun_live") and not self.physical_enabled:
            raise PermissionError("PHYSICAL_EXECUTION_DISABLED")
        return True

    def run(self, mode, human_confirmation, handlers):
        """Authorize, then invoke the explicitly supplied mode adapter."""
        self.authorize(mode, human_confirmation)
        if mode not in handlers:
            raise RuntimeError("MODE_ADAPTER_NOT_BOUND")
        return handlers[mode]()

    @staticmethod
    def mode_contract(mode):
        return {
          "dryrun_offline":"files only",
          "dryrun_live":"feedback/camera read only; commands file only",
          "gripper_test":"arm stationary; OPEN and CLOSE once",
          "stop_test_movej":"from real_home, MoveJoint J6 +2 deg at <=5 deg/s, then MoveStop",
          "stop_test_servoj":"small Servoj joint motion, then the same MoveStop mode",
          "synthetic":"5 mm translation / 1 deg rotation axes",
          "policy":"mode A arm only; gripper record only",
        }[mode]

    @staticmethod
    def required_settings(mode):
        if mode not in MODE_REQUIRED_SETTINGS: raise ValueError("unknown mode")
        return MODE_REQUIRED_SETTINGS[mode]
