
SET(UNITTEST 0 CACHE BOOL "tuw_control unittests" )
IF(UNITTEST)

include_directories(${GTEST_INCLUDE_DIR})
enable_testing()

add_executable(test_param_func_spline0 test_param_func_manip_spline0.cpp)
target_link_libraries(test_param_func_spline0 gtest tuw_control_param_func_spline ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_param_func_spline0_dist test_param_func_manip_spline0_dist.cpp)
target_link_libraries(test_param_func_spline0_dist gtest ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_simulators test_trajectory_simulators.cpp)
target_link_libraries(test_trajectory_simulators gtest tuw_trajectory_simulator ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_simulator test_trajectory_simulator.cpp)
target_link_libraries(test_trajectory_simulator gtest ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_simulator_new test_trajectory_simulator_new.cpp)
target_link_libraries(test_trajectory_simulator_new gtest ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_sim_grade test_trajectory_sim_grade.cpp)
target_link_libraries(test_trajectory_sim_grade gtest tuw_trajectory_simulator tuw_trajectory_evaluator ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_optimizer test_trajectory_optimizer.cpp)
target_link_libraries(test_trajectory_optimizer gtest tuw_trajectory_simulator tuw_trajectory_evaluator ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_state_map test_state_map.cpp)
target_link_libraries(test_state_map gtest ${CMAKE_THREAD_LIBS_INIT})

add_test(
    NAME test_param_func_spline0
    COMMAND test_param_func_spline0
)
add_test(
    NAME test_param_func_spline0_dist
    COMMAND test_param_func_spline0_dist
)

add_test(
    NAME test_trajectory_simulators
    COMMAND test_trajectory_simulators
)

add_test(
    NAME test_trajectory_simulator
    COMMAND test_trajectory_simulator
)

add_test(
    NAME test_trajectory_simulator_new
    COMMAND test_trajectory_simulator_new
)

add_test(
    NAME test_trajectory_sim_grade
    COMMAND test_trajectory_sim_grade
)
add_test(
    NAME test_trajectory_optimizer
    COMMAND test_trajectory_optimizer
)
add_test(
    NAME test_state_map
    COMMAND test_state_map
)


ENDIF(UNITTEST)
