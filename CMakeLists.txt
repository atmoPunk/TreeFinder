cmake_minimum_required(VERSION 3.18)
project(TreeFinder)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS program_options REQUIRED )

add_subdirectory(libs)


include_directories(${Boost_INCLUDE_DIRS})

add_executable(TreeFinder src/main.cpp)

target_link_libraries( TreeFinder libtreefinder )
target_link_libraries( TreeFinder libimages )
target_link_libraries( TreeFinder ${Boost_LIBRARIES})
