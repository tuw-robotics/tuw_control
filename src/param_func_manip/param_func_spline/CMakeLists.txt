set(FILES
  param_func_spline0.cpp
  param_func_spline0_dist.cpp
)

add_library( tuw_control_param_func_spline SHARED ${FILES})
target_link_libraries( tuw_control_param_func_spline tuw_control_param_func )
