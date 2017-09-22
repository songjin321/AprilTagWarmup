cmake_minimum_required(VERSION 2.6)

# Include our headers
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# find all source and header files
file(GLOB SRCS RELATIVE "${PROJECT_SOURCE_DIR}/thirdparty/apriltags/" "*.cc")

# apriltags
add_library(apriltags ${SRCS})

# Find opencv and build our binary
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltags ${OpenCV_LIBS}) #-pg) #-fopenmp)
