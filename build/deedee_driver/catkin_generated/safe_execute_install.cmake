execute_process(COMMAND "/home/cyrill/ros-playground/build/deedee_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cyrill/ros-playground/build/deedee_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
