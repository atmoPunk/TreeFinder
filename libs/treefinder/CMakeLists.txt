cmake_minimum_required(VERSION 3.18)

project(libtreefinder)

set(HEADERS
        libtreefinder/treefinder.h
        libtreefinder/interpolation.h
        )

set(SOURCES
        libtreefinder/treefinder.cpp
        libtreefinder/interpolation.cpp
        )

set(CMAKE_CXX_STANDARD 20)

find_package( OpenCV REQUIRED )

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} libimages)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
