add_library( state_map SHARED state_map.cpp)
add_executable( demo_state_map demo_state_map.cpp)
target_link_libraries( demo_state_map  state_map)