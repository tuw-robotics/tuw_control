set(FILES
  trajectory_simulator.cpp
  trajectory_simulator_online.cpp
  trajectory_simulator_precalc.cpp
)
add_library( tuw_trajectory_simulator SHARED ${FILES})
target_link_libraries( tuw_trajectory_simulator tuw_control_param_func_spline )


## Mark executables and/or libraries for installation
install(TARGETS tuw_trajectory_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
