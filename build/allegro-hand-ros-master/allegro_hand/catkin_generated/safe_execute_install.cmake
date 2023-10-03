execute_process(COMMAND "/home/joe/sg_ws/build/allegro-hand-ros-master/allegro_hand/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/joe/sg_ws/build/allegro-hand-ros-master/allegro_hand/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
