cmake_minimum_required(VERSION 3.18)
project(TreeFinder)

set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_BUILD_TYPE Debug)

find_package( OpenCV REQUIRED )

add_subdirectory(libs)

add_executable(TreeFinder src/main.cpp)
target_link_libraries( TreeFinder libtreefinder )
target_link_libraries( TreeFinder libimages )
