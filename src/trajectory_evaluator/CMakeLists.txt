set(FILES
  trajectory_sim_grade.cpp
  trajectory_optimizer.cpp
)
add_library( tuw_trajectory_evaluator SHARED ${FILES})
target_link_libraries( tuw_trajectory_evaluator tuw_trajectory_simulator )

## Mark executables and/or libraries for installation
install(TARGETS tuw_trajectory_evaluator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
