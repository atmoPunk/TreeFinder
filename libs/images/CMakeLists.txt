cmake_minimum_required(VERSION 3.1)

project(libimages)

# This is thin wrapper around CImg header-only library available at https://github.com/dtschump/CImg (v.2.3.4 0f0d65d984b08ad8178969f4fa4d1641d721354b)

set(HEADERS
        libimages/images.h
        )

set(SOURCES
        libimages/images.cpp
        )

set(CMAKE_CXX_STANDARD 11)

set(LIBRARIES)

if(UNIX)
    find_package(X11 REQUIRED)
    set(LIBRARIES ${LIBRARIES} ${X11_LIBRARIES})
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(TIFF REQUIRED)
set(LIBRARIES ${LIBRARIES} ${TIFF_LIBRARIES})

add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} Threads::Threads ${LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
