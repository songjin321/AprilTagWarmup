cmake_minimum_required(VERSION 2.8.3)
project(apriltag_localization)

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
roscpp
tf
std_msgs
nav_msgs
apriltag_checkout_tag
apriltag_mapping
)
#pkg_search_module(Eigen3 REQUIRED eigen3)

# #debug
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#   message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
# #end debug

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs tf std_msgs apriltag_checkout_tag apriltag_mapping
  DEPENDS Eigen3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

#add_subdirectory(thirdparty)

add_executable(apriltag_localization 
src/tmcl.cpp
src/tmcl_node.cpp
)

target_link_libraries(apriltag_localization ${catkin_LIBRARIES} yaml-cpp glog gflags_nothreads)