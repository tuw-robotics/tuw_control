set(FILES
  param_func.cpp
)
add_library( tuw_control_param_func SHARED ${FILES})
target_link_libraries( tuw_control_param_func )


## Mark executables and/or libraries for installation
install(TARGETS tuw_control_param_func
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(param_func_spline)
