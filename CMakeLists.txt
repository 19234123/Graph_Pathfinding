cmake_minimum_required(VERSION 3.26)
project(GraphPathfinding)

set(CMAKE_CXX_STANDARD 17)

# Set the source files for the library
set(SOURCE_FILES C:/Dev/Libraries/GraphPathfinding/Graph.cpp)

# Create the library
add_library(graph STATIC ${SOURCE_FILES})

# Include directories for the library
target_include_directories(graph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)