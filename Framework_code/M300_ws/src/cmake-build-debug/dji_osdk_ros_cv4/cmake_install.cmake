# Install script for directory: /home/erfan/M300_ws/src/dji_osdk_ros_cv4

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_osdk_ros/msg" TYPE FILE FILES
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FCTimeInUTC.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/JoystickParams.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/GPSUTC.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Gimbal.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/Waypoint.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointList.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorFlying.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuatorRotateHeading.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AircraftControlActuator.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2AssociateTrigger.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocalLengthParam.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuatorFocusParam.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2CameraActuator.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Config.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuatorRotationParam.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2GimbalActuator.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2InitSetting.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2IntervalTrigger.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2ReachpointTrigger.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2SampleReachPointTrigger.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2TrajectoryTrigger.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2Action.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionEventPush.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/WaypointV2MissionStatePush.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MobileData.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/PayloadData.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointAction.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypoint.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionWaypointTask.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/MissionHotpointTask.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/FlightAnomaly.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/VOPosition.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryState.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/BatteryWholeInfo.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryState.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/SmartBatteryDynamicInfo.msg"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/msg/HMSPushInfo.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_osdk_ros/srv" TYPE FILE FILES
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetDroneType.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetM300StereoParams.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/FlightTaskControl.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetJoystickMode.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/JoystickAction.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ObtainControlAuthority.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/KillSwitch.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/EmergencyBrake.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GimbalAction.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraEV.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraShutterSpeed.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAperture.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraISO.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraFocusPoint.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraTapZoomPoint.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraSetZoomPara.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraZoomCtrl.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootSinglePhoto.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootBurstPhoto.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootAEBPhoto.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStartShootIntervalPhoto.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraStopShootPhoto.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraRecordVideoAction.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetWholeBatteryInfo.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetSingleBatteryDynamicInfo.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIO.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGoHomeAltitude.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGoHomeAltitude.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetCurrentAircraftLocAsHomePoint.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHomePoint.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraH264.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetAvoidEnable.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetAvoidEnable.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/InitWaypointV2Setting.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Mission.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/UploadWaypointV2Action.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DownloadWaypointV2Mission.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StartWaypointV2Mission.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StopWaypointV2Mission.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/PauseWaypointV2Mission.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/ResumeWaypointV2Mission.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GenerateWaypointV2Action.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetGlobalCruisespeed.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetGlobalCruisespeed.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/GetHMSData.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2Event.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SubscribeWaypointV2State.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Activation.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/CameraAction.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneTaskControl.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SDKControlAuthority.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetLocalPosRef.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOConfig.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MFIOSetValue.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/DroneArmControl.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionStatus.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpAction.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpAction.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpUpload.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpSetSpeed.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetSpeed.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionWpGetInfo.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpload.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpGetInfo.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateYawRate.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpUpdateRadius.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/MissionHpResetYaw.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendMobileData.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SendPayloadData.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetHardSync.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/QueryDroneVersion.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/Stereo240pSubscription.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoVGASubscription.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/StereoDepthSubscription.srv"
    "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/srv/SetupCameraStream.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_osdk_ros/cmake" TYPE FILE FILES "/home/erfan/M300_ws/src/cmake-build-debug/dji_osdk_ros_cv4/catkin_generated/installspace/dji_osdk_ros-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/erfan/M300_ws/src/cmake-build-debug/devel/include/dji_osdk_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/erfan/M300_ws/src/cmake-build-debug/devel/share/roseus/ros/dji_osdk_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/erfan/M300_ws/src/cmake-build-debug/devel/share/common-lisp/ros/dji_osdk_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/erfan/M300_ws/src/cmake-build-debug/devel/share/gennodejs/ros/dji_osdk_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/erfan/M300_ws/src/cmake-build-debug/devel/lib/python3/dist-packages/dji_osdk_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/erfan/M300_ws/src/cmake-build-debug/devel/lib/python3/dist-packages/dji_osdk_ros")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/erfan/M300_ws/src/cmake-build-debug/dji_osdk_ros_cv4/catkin_generated/installspace/dji_osdk_ros.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_osdk_ros/cmake" TYPE FILE FILES "/home/erfan/M300_ws/src/cmake-build-debug/dji_osdk_ros_cv4/catkin_generated/installspace/dji_osdk_ros-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_osdk_ros/cmake" TYPE FILE FILES
    "/home/erfan/M300_ws/src/cmake-build-debug/dji_osdk_ros_cv4/catkin_generated/installspace/dji_osdk_rosConfig.cmake"
    "/home/erfan/M300_ws/src/cmake-build-debug/dji_osdk_ros_cv4/catkin_generated/installspace/dji_osdk_rosConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_osdk_ros" TYPE FILE FILES "/home/erfan/M300_ws/src/dji_osdk_ros_cv4/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/erfan/M300_ws/src/cmake-build-debug/dji_osdk_ros_cv4/src/cmake_install.cmake")

endif()

