cmake_minimum_required(VERSION 2.8.3)
project(tuw_control)
include(macros.cmake)
_version(2016 04 01)
_set_output_paths()
_installation_destinations() # must be befor add_subdirectory

add_compile_options(-std=c++1y  -Wno-reorder) ## for c++ 2011

SET(CMAKE_BUILD_TYPE Release)

MESSAGE( STATUS "CMAKE_BUILD_TYPE:              " ${CMAKE_BUILD_TYPE} )
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -fno-strength-reduce -pipe ")# -Og
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -pipe -Ofast")


if(CMAKE_BUILD_TYPE MATCHES Release)
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CSS_FLAGS_RELEASE} )
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES Debug)
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG:         " ${CMAKE_CXX_FLAGS_DEBUG} )
endif(CMAKE_BUILD_TYPE MATCHES Debug)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer date_time program_options system thread iostreams regex signals date_time serialization filesystem )
find_package(OpenCV REQUIRED)
 
find_file(MATHGL_HEADER mgl2/qt.h PATHS)
IF(MATHGL_HEADER)
  SET(USE_MATHGL 1)
  add_definitions( -DUSE_MATHGL )
  set(MATHGL_LIBRARIES mgl-qt mgl)
  MESSAGE( STATUS "MATHGL found" )
else()
  MESSAGE( STATUS ${MATHGL_HEADER})
  MESSAGE( STATUS "--> MathGL: Ubuntu 14.04: \"sudo apt-get install mathgl-dev\" ")
endif()


generate_dynamic_reconfigure_options(
)

catkin_package(
  INCLUDE_DIRS 
  include
  
  LIBRARIES 
  tuw_control_param_func_spline
  tuw_control_filters
  
  DEPENDS 
  OpenCV
)

include_directories(${catkin_INCLUDE_DIRS} ./include/)
add_subdirectory(src)
add_subdirectory(tests)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

_generate_find_package() # must be after add_subdirectory
_uninstall()    
_doxygen()