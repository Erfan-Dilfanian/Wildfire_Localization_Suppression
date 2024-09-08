# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "forest_fire_detection_system: 3 messages, 2 services")

set(MSG_I_FLAGS "-Iforest_fire_detection_system:/home/erfan/M300_ws/src/forest_fire_detection_system/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(forest_fire_detection_system_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" NAME_WE)
add_custom_target(_forest_fire_detection_system_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "forest_fire_detection_system" "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" NAME_WE)
add_custom_target(_forest_fire_detection_system_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "forest_fire_detection_system" "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" NAME_WE)
add_custom_target(_forest_fire_detection_system_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "forest_fire_detection_system" "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" NAME_WE)
add_custom_target(_forest_fire_detection_system_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "forest_fire_detection_system" "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" ""
)

get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" NAME_WE)
add_custom_target(_forest_fire_detection_system_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "forest_fire_detection_system" "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_cpp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_cpp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Services
_generate_srv_cpp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
)
_generate_srv_cpp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Module File
_generate_module_cpp(forest_fire_detection_system
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(forest_fire_detection_system_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(forest_fire_detection_system_generate_messages forest_fire_detection_system_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_cpp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_cpp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_cpp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_cpp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_cpp _forest_fire_detection_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(forest_fire_detection_system_gencpp)
add_dependencies(forest_fire_detection_system_gencpp forest_fire_detection_system_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS forest_fire_detection_system_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_eus(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_eus(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Services
_generate_srv_eus(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
)
_generate_srv_eus(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Module File
_generate_module_eus(forest_fire_detection_system
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(forest_fire_detection_system_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(forest_fire_detection_system_generate_messages forest_fire_detection_system_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_eus _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_eus _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_eus _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_eus _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_eus _forest_fire_detection_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(forest_fire_detection_system_geneus)
add_dependencies(forest_fire_detection_system_geneus forest_fire_detection_system_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS forest_fire_detection_system_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_lisp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_lisp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Services
_generate_srv_lisp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
)
_generate_srv_lisp(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Module File
_generate_module_lisp(forest_fire_detection_system
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(forest_fire_detection_system_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(forest_fire_detection_system_generate_messages forest_fire_detection_system_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_lisp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_lisp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_lisp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_lisp _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_lisp _forest_fire_detection_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(forest_fire_detection_system_genlisp)
add_dependencies(forest_fire_detection_system_genlisp forest_fire_detection_system_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS forest_fire_detection_system_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_nodejs(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_nodejs(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Services
_generate_srv_nodejs(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
)
_generate_srv_nodejs(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Module File
_generate_module_nodejs(forest_fire_detection_system
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(forest_fire_detection_system_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(forest_fire_detection_system_generate_messages forest_fire_detection_system_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_nodejs _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_nodejs _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_nodejs _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_nodejs _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_nodejs _forest_fire_detection_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(forest_fire_detection_system_gennodejs)
add_dependencies(forest_fire_detection_system_gennodejs forest_fire_detection_system_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS forest_fire_detection_system_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_py(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
)
_generate_msg_py(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Services
_generate_srv_py(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
)
_generate_srv_py(forest_fire_detection_system
  "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
)

### Generating Module File
_generate_module_py(forest_fire_detection_system
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(forest_fire_detection_system_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(forest_fire_detection_system_generate_messages forest_fire_detection_system_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireFuse.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_py _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireIR.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_py _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/msg/SingleFireRGB.msg" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_py _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/SegmentSmoke.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_py _forest_fire_detection_system_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/erfan/M300_ws/src/forest_fire_detection_system/srv/ToggleGrabDataDepthEstimation.srv" NAME_WE)
add_dependencies(forest_fire_detection_system_generate_messages_py _forest_fire_detection_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(forest_fire_detection_system_genpy)
add_dependencies(forest_fire_detection_system_genpy forest_fire_detection_system_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS forest_fire_detection_system_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/forest_fire_detection_system
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(forest_fire_detection_system_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/forest_fire_detection_system
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(forest_fire_detection_system_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/forest_fire_detection_system
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(forest_fire_detection_system_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/forest_fire_detection_system
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(forest_fire_detection_system_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/forest_fire_detection_system
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(forest_fire_detection_system_generate_messages_py std_msgs_generate_messages_py)
endif()
