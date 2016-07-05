set(FILES
  ekf.cpp
)
add_library( tuw_control_filters SHARED ${FILES})
target_link_libraries( tuw_control_filters ${OpenCV_LIBRARIES})
