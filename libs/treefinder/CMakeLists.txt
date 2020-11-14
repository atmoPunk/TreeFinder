cmake_minimum_required(VERSION 3.18)

project(libtreefinder)

set(HEADERS
        libtreefinder/treefinder.h
        libtreefinder/interpolation.h
        libtreefinder/tree_deliniation.h
        )

set(SOURCES
        libtreefinder/treefinder.cpp
        libtreefinder/interpolation.cpp
        libtreefinder/tree_deliniation.cpp
        )

set(CMAKE_CXX_STANDARD 20)

find_package( OpenCV REQUIRED )
find_package( Boost REQUIRED )

include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} libimages)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
