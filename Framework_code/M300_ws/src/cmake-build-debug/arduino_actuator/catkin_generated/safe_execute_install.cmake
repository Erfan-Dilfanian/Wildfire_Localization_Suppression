execute_process(COMMAND "/home/erfan/M300_ws/src/cmake-build-debug/arduino_actuator/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/erfan/M300_ws/src/cmake-build-debug/arduino_actuator/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
