# Getter source audit

No getter was invoked: none meets SAFE_READ_ONLY live-stability evidence.
GetCurrentTcp/GetCurrentTool return only string info; success=true is hardcoded, not an independent SDK success check.

## dsr_msgs2/srv/GetCurrentTcp

```json
{
  "type": "dsr_msgs2/srv/GetCurrentTcp",
  "request": {},
  "response": {
    "info": "string",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/tcp/GetCurrentTcp.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 1938,
  "callback": "auto get_current_tcp_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentTcp::Response> res) -> void    \n{\n    //ROS_INFO(\"get_current_tcp_cb() called and calling Drfl->get_tcp\");\n    res->info = Drfl->get_tcp();\n    res->success = true;\n\n};",
  "service_registration": [
    "  m_nh_srv_get_current_tcp        = get_node()->create_service<dsr_msgs2::srv::GetCurrentTcp>(\"tcp/get_current_tcp\", get_current_tcp_cb);       "
  ],
  "drfl": "get_tcp",
  "wrapper": [
    "string get_tcp() { return string(_get_tcp(_rbtCtrl)); };",
    "DRFL_DEPRECATED(\"Deprecated: Use get_tcp() instead.\")"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetCurrentTool

```json
{
  "type": "dsr_msgs2/srv/GetCurrentTool",
  "request": {},
  "response": {
    "info": "string",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/tool/GetCurrentTool.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 1967,
  "callback": "auto get_current_tool_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetCurrentTool::Response> res) -> void \n{\n    //ROS_INFO(\"get_current_tool_cb() called and calling Drfl->get_tool %s\", Drfl->GetCurrentTool().c_str());\n    res->info = Drfl->get_tool();\n    res->success = true;\n};",
  "service_registration": [
    "  m_nh_srv_get_current_tool       = get_node()->create_service<dsr_msgs2::srv::GetCurrentTool>(\"tool/get_current_tool\", get_current_tool_cb);     "
  ],
  "drfl": "get_tool",
  "wrapper": [
    "string get_tool() { return string(_get_tool(_rbtCtrl)); };",
    "DRFL_DEPRECATED(\"Deprecated: Use get_tool() instead.\")"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetCurrentPosx

```json
{
  "type": "dsr_msgs2/srv/GetCurrentPosx",
  "request": {
    "ref": "int8"
  },
  "response": {
    "task_pos_info": "sequence<std_msgs/Float64MultiArray>",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/aux_control/GetCurrentPosx.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 983,
  "callback": "auto get_current_posx_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPosx::Response> res)-> void                             \n{\n    dsr_instrumentation::Scope trace_scope(\"get_current_posx_cb\");\n    dsr_instrumentation::trace(\"GET_CURRENT_POSX_REQUEST\", \"get_current_posx_cb\", \"ref=%d\", static_cast<int>(req->ref));\n    std_msgs::msg::Float64MultiArray arr;\n\n#if (_DEBUG_DSR_CTL)\n    RCLCPP_INFO(rclcpp::get_logger(\"dsr_controller2\"),\"< get_current_posx_cb >\");\n#endif\n\n    LPROBOT_TASK_POSE cur_posx = Drfl->get_current_posx((COORDINATE_SYSTEM)req->ref);\n    dsr_instrumentation::trace(\"GET_CURRENT_POSX_RESULT\", \"get_current_posx_cb\", \"result=%p\", cur_posx);\n    if(nullptr == cur_posx) {\n        res->success = false;\n        return;\n    }\n    arr.data.clear();\n    for (int i = 0; i < NUM_TASK; i++){\n        arr.data.push_back(cur_posx->_fTargetPos[i]);\n    }\n    arr.data.push_back(cur_posx->_iTargetSol);\n    res->task_pos_info.push_back(arr);\n    res->success = true;\n};",
  "service_registration": [
    "  m_nh_srv_get_current_posx               = get_node()->create_service<dsr_msgs2::srv::GetCurrentPosx>(\"aux_control/get_current_posx\", get_current_posx_cb);                               "
  ],
  "drfl": "get_current_posx",
  "wrapper": [
    "LPROBOT_TASK_POSE get_current_posx(COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetCurrentPose

```json
{
  "type": "dsr_msgs2/srv/GetCurrentPose",
  "request": {
    "space_type": "int8"
  },
  "response": {
    "pos": "double[6]",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/system/GetCurrentPose.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 370,
  "callback": "auto get_current_pose_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Request> req, std::shared_ptr<dsr_msgs2::srv::GetCurrentPose::Response> res)-> void\n{\n    RCLCPP_INFO(rclcpp::get_logger(\"dsr_controller2\"),\"get_current_pose_cb() called and calling Drfl->get_current_pose(%d)\",req->space_type);\n\n    LPROBOT_POSE robot_pos = Drfl->get_current_pose((ROBOT_SPACE)req->space_type);\n    for(int i = 0; i < NUM_TASK; i++){\n        res->pos[i] = robot_pos->_fPosition[i];\n    }\n    res->success = true;\n};",
  "service_registration": [
    "  m_nh_srv_get_current_pose           = get_node()->create_service<dsr_msgs2::srv::GetCurrentPose>(\"system/get_current_pose\", get_current_pose_cb);   "
  ],
  "drfl": "get_current_pose",
  "wrapper": [
    "LPROBOT_POSE get_current_pose(ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT) { return _get_current_pose(_rbtCtrl, eSpaceType); };",
    "DRFL_DEPRECATED(\"Deprecated: Use get_current_pose() instead.\")"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetRobotMode

```json
{
  "type": "dsr_msgs2/srv/GetRobotMode",
  "request": {},
  "response": {
    "robot_mode": "int8",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/system/GetRobotMode.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 326,
  "callback": "auto get_robot_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotMode::Response> res)-> void\n{       \n    RCLCPP_INFO(rclcpp::get_logger(\"dsr_controller2\"),\"get_robot_mode_cb() called and calling Drfl->get_robot_mode()\");\n    res->robot_mode = Drfl->get_robot_mode();\n    res->success = true;\n};",
  "service_registration": [
    "  m_nh_srv_get_robot_mode             = get_node()->create_service<dsr_msgs2::srv::GetRobotMode>(\"system/get_robot_mode\", get_robot_mode_cb);     "
  ],
  "drfl": "get_robot_mode",
  "wrapper": [
    "ROBOT_MODE get_robot_mode() { return _get_robot_mode(_rbtCtrl); };",
    "DRFL_DEPRECATED(\"Deprecated: Use get_robot_mode() instead.\")"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetRobotState

```json
{
  "type": "dsr_msgs2/srv/GetRobotState",
  "request": {},
  "response": {
    "robot_state": "int8",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/system/GetRobotState.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 347,
  "callback": "auto get_robot_state_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotState::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotState::Response> res)-> void\n{\n    RCLCPP_INFO(rclcpp::get_logger(\"dsr_controller2\"),\"get_robot_state_cb() called and calling Drfl->get_robot_state()\");\n\n    res->robot_state = Drfl->get_robot_state();\n    res->success = true;\n};",
  "service_registration": [
    "  m_nh_srv_get_robot_state            = get_node()->create_service<dsr_msgs2::srv::GetRobotState>(\"system/get_robot_state\", get_robot_state_cb);        "
  ],
  "drfl": "get_robot_state",
  "wrapper": [
    "ROBOT_STATE get_robot_state() { return _get_robot_state(_rbtCtrl); };",
    "DRFL_DEPRECATED(\"Deprecated: Use get_robot_state() instead.\")"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetRobotSystem

```json
{
  "type": "dsr_msgs2/srv/GetRobotSystem",
  "request": {},
  "response": {
    "robot_system": "int8",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/system/GetRobotSystem.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 339,
  "callback": "auto get_robot_system_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetRobotSystem::Response> res)-> void\n{\n    RCLCPP_INFO(rclcpp::get_logger(\"dsr_controller2\"),\"get_robot_system_cb() called and calling Drfl->get_robot_system()\");\n\n    res->robot_system = Drfl->get_robot_system();\n    res->success = true;\n};",
  "service_registration": [
    "  m_nh_srv_get_robot_system           = get_node()->create_service<dsr_msgs2::srv::GetRobotSystem>(\"system/get_robot_system\", get_robot_system_cb);         "
  ],
  "drfl": "get_robot_system",
  "wrapper": [
    "ROBOT_SYSTEM get_robot_system() { return _get_robot_system(_rbtCtrl); };",
    "DRFL_DEPRECATED(\"Deprecated: Use get_robot_system() instead.\")"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```

## dsr_msgs2/srv/GetControlMode

```json
{
  "type": "dsr_msgs2/srv/GetControlMode",
  "request": {},
  "response": {
    "control_mode": "int8",
    "success": "boolean"
  },
  "source_srv": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_msgs2/srv/aux_control/GetControlMode.srv",
  "callback_file": "/home/ubuntu/robot_ws/src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp",
  "callback_line": 913,
  "callback": "auto get_control_mode_cb = [this](const std::shared_ptr<dsr_msgs2::srv::GetControlMode::Request> /*req*/, std::shared_ptr<dsr_msgs2::srv::GetControlMode::Response> res)-> void                         \n{\n    res->success = false;\n#if (_DEBUG_DSR_CTL)\n    RCLCPP_INFO(rclcpp::get_logger(\"dsr_controller2\"),\"< get_control_mode_cb >\");\n#endif\n    //NO API , get mon_data      \n    res->control_mode = g_stDrState.nActualMode;\n    res->success = true;       \n};",
  "service_registration": [
    "  m_nh_srv_get_control_mode               = get_node()->create_service<dsr_msgs2::srv::GetControlMode>(\"aux_control/get_control_mode\", get_control_mode_cb);                           "
  ],
  "drfl": "get_control_mode",
  "wrapper": [
    "CONTROL_MODE get_control_mode(){ return _get_control_mode(_rbtCtrl);};"
  ],
  "classification": "READ_ONLY_BUT_LIVE_STABILITY_UNVERIFIED",
  "called": false,
  "reason": "Visible callback is getter-only; vendor implementation/stability not proven. Posx has prior feedback-loss correlation."
}
```