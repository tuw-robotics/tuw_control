
SET(UNITTEST 0 CACHE BOOL "tuw_control unittests" )
IF(UNITTEST)

include_directories(${GTEST_INCLUDE_DIR})
enable_testing()

add_executable(test_param_func_spline0 test_param_func_manip_spline0.cpp)
target_link_libraries(test_param_func_spline0 gtest tuw_control_param_func_spline ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_simulators test_trajectory_simulators.cpp)
target_link_libraries(test_trajectory_simulators gtest tuw_trajectory_simulator ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_trajectory_sim_grade test_trajectory_sim_grade.cpp)
target_link_libraries(test_trajectory_sim_grade gtest tuw_trajectory_simulator tuw_trajectory_evaluator ${CMAKE_THREAD_LIBS_INIT})

add_test(
    NAME test_param_func_spline0
    COMMAND test_param_func_spline0
)

add_test(
    NAME test_trajectory_simulators
    COMMAND test_trajectory_simulators
)
add_test(
    NAME test_trajectory_sim_grade
    COMMAND test_trajectory_sim_grade
)



ENDIF(UNITTEST)
