# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dji_osdk_ros: 38 messages, 75 services")

set(MSG_I_FLAGS "-Idji_osdk_ros:/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dji_osdk_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" "dji_osdk_ros/Waypoint"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" "dji_osdk_ros/WaypointV2AircraftControlActuatorRotateHeading:dji_osdk_ros/WaypointV2AircraftControlActuatorFlying"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" "dji_osdk_ros/WaypointV2CameraActuatorFocusParam:dji_osdk_ros/WaypointV2CameraActuatorFocalLengthParam"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" "dji_osdk_ros/WaypointV2GimbalActuatorRotationParam"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" "dji_osdk_ros/WaypointV2:dji_osdk_ros/WaypointV2Config"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" "dji_osdk_ros/WaypointV2AircraftControlActuator:dji_osdk_ros/WaypointV2AircraftControlActuatorFlying:dji_osdk_ros/WaypointV2IntervalTrigger:dji_osdk_ros/WaypointV2CameraActuatorFocusParam:dji_osdk_ros/WaypointV2CameraActuatorFocalLengthParam:dji_osdk_ros/WaypointV2GimbalActuator:dji_osdk_ros/WaypointV2SampleReachPointTrigger:dji_osdk_ros/WaypointV2GimbalActuatorRotationParam:dji_osdk_ros/WaypointV2TrajectoryTrigger:dji_osdk_ros/WaypointV2AircraftControlActuatorRotateHeading:dji_osdk_ros/WaypointV2AssociateTrigger:dji_osdk_ros/WaypointV2CameraActuator"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" "dji_osdk_ros/WaypointV2Config"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" "dji_osdk_ros/MissionWaypointAction"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" "dji_osdk_ros/MissionWaypoint:dji_osdk_ros/MissionWaypointAction"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" "dji_osdk_ros/BatteryState"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" "dji_osdk_ros/SmartBatteryState"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" "dji_osdk_ros/JoystickParams"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" "dji_osdk_ros/JoystickParams"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" "std_msgs/Header"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" "dji_osdk_ros/BatteryWholeInfo:dji_osdk_ros/BatteryState"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" "dji_osdk_ros/SmartBatteryState:dji_osdk_ros/SmartBatteryDynamicInfo"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" "dji_osdk_ros/WaypointV2InitSetting:dji_osdk_ros/WaypointV2:dji_osdk_ros/WaypointV2Config"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" "dji_osdk_ros/WaypointV2:dji_osdk_ros/WaypointV2Config"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" "dji_osdk_ros/WaypointV2AircraftControlActuator:dji_osdk_ros/WaypointV2AircraftControlActuatorFlying:dji_osdk_ros/WaypointV2IntervalTrigger:dji_osdk_ros/WaypointV2CameraActuatorFocusParam:dji_osdk_ros/WaypointV2CameraActuatorFocalLengthParam:dji_osdk_ros/WaypointV2GimbalActuator:dji_osdk_ros/WaypointV2Action:dji_osdk_ros/WaypointV2SampleReachPointTrigger:dji_osdk_ros/WaypointV2GimbalActuatorRotationParam:dji_osdk_ros/WaypointV2TrajectoryTrigger:dji_osdk_ros/WaypointV2AircraftControlActuatorRotateHeading:dji_osdk_ros/WaypointV2AssociateTrigger:dji_osdk_ros/WaypointV2CameraActuator"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" "dji_osdk_ros/HMSPushInfo"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" "dji_osdk_ros/MissionWaypoint:dji_osdk_ros/MissionWaypointTask:dji_osdk_ros/MissionWaypointAction"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" "dji_osdk_ros/MissionWaypoint:dji_osdk_ros/MissionWaypointTask:dji_osdk_ros/MissionWaypointAction"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" "dji_osdk_ros/MissionHotpointTask"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" "dji_osdk_ros/MissionHotpointTask"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" NAME_WE)
add_custom_target(_dji_osdk_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dji_osdk_ros" "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)

### Generating Services
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_cpp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
)

### Generating Module File
_generate_module_cpp(dji_osdk_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dji_osdk_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dji_osdk_ros_generate_messages dji_osdk_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_cpp _dji_osdk_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_osdk_ros_gencpp)
add_dependencies(dji_osdk_ros_gencpp dji_osdk_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_osdk_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)

### Generating Services
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_eus(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
)

### Generating Module File
_generate_module_eus(dji_osdk_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dji_osdk_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dji_osdk_ros_generate_messages dji_osdk_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_eus _dji_osdk_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_osdk_ros_geneus)
add_dependencies(dji_osdk_ros_geneus dji_osdk_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_osdk_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)

### Generating Services
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_lisp(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
)

### Generating Module File
_generate_module_lisp(dji_osdk_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dji_osdk_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dji_osdk_ros_generate_messages dji_osdk_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_lisp _dji_osdk_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_osdk_ros_genlisp)
add_dependencies(dji_osdk_ros_genlisp dji_osdk_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_osdk_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)

### Generating Services
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_nodejs(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
)

### Generating Module File
_generate_module_nodejs(dji_osdk_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dji_osdk_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dji_osdk_ros_generate_messages dji_osdk_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_nodejs _dji_osdk_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_osdk_ros_gennodejs)
add_dependencies(dji_osdk_ros_gennodejs dji_osdk_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_osdk_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_msg_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)

### Generating Services
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg;/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)
_generate_srv_py(dji_osdk_ros
  "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
)

### Generating Module File
_generate_module_py(dji_osdk_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dji_osdk_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dji_osdk_ros_generate_messages dji_osdk_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv" NAME_WE)
add_dependencies(dji_osdk_ros_generate_messages_py _dji_osdk_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dji_osdk_ros_genpy)
add_dependencies(dji_osdk_ros_genpy dji_osdk_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dji_osdk_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dji_osdk_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dji_osdk_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(dji_osdk_ros_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dji_osdk_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(dji_osdk_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dji_osdk_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dji_osdk_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(dji_osdk_ros_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dji_osdk_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(dji_osdk_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dji_osdk_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dji_osdk_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(dji_osdk_ros_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dji_osdk_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(dji_osdk_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dji_osdk_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dji_osdk_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(dji_osdk_ros_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dji_osdk_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(dji_osdk_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dji_osdk_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dji_osdk_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(dji_osdk_ros_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dji_osdk_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(dji_osdk_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
