set(FILES
  param_func.cpp
)
add_library( tuw_control_param_func SHARED ${FILES})
target_link_libraries( tuw_control_param_func )


add_subdirectory(param_func_spline)
