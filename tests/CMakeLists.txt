
SET(UNITTEST 0 CACHE BOOL "tuw_control unittests" )
IF(UNITTEST)

include_directories(${GTEST_INCLUDE_DIR})
enable_testing()

add_executable(test_param_func_spline0 test_param_func_manip_spline0.cpp)
target_link_libraries(test_param_func_spline0 gtest param_func_spline ${CMAKE_THREAD_LIBS_INIT})

add_test(
    NAME test_param_func_spline0
    COMMAND test_param_func_spline0
)

ENDIF(UNITTEST)
