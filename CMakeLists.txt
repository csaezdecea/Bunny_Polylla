cmake_minimum_required(VERSION 3.16)
project(CGALPOL)
set(CMAKE_CXX_STANDARD 17)

# Specify output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# Include directory for header files
include_directories(include)
include_directories(${CGAL_INCLUDE_DIRS})

#Add Excutables
add_executable(Bunny src/bunny.cpp)
add_executable(VHedges src/vhalfedges.cpp)



find_package(CGAL REQUIRED COMPONENTS Core)
#find_package(CGAL REQUIRED COMPONENTS Polygon_mesh_processing)
#target_link_libraries(your_executable PRIVATE CGAL::Polygon_mesh_processing)

target_link_libraries(Bunny CGAL::CGAL CGAL::CGAL_Core)
target_link_libraries(VHedges CGAL::CGAL CGAL::CGAL_Core)

#find_package(CGAL REQUIRED COMPONENTS Polygon_mesh_processing)
#include_directories(${CGAL_INCLUDE_DIRS})
#target_link_libraries(SphVor PRIVATE CGAL::Polygon_mesh_processing)

#set(CGAL_DATA_DIR "./data")
#set(CGAL_DATA_DIR "${CMAKE_SOURCE_DIR}/../data")