cmake_minimum_required(VERSION 3.10)
project(NslApp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# option BUILD_PCL : default OFF
option(BUILD_PCL "Build as PCL library" OFF)

if (BUILD_PCL)
find_package(PCL 1.2 REQUIRED COMPONENTS common visualization)
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS nsl_lib/include REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/linux-7.5/shared REQUIRED)
find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/aarch-7.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/windows/shared REQUIRED)

add_compile_options(-Wno-deprecated-declarations)

add_executable(nslApp main.cpp)

if (BUILD_PCL)
target_link_libraries(nslApp PRIVATE ${NSL_LIB_PATH} ${OpenCV_LIBS} ${PCL_LIBRARIES} pthread)
target_include_directories(nslApp PRIVATE ${NSL_LIB_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
else()
target_link_libraries(nslApp PRIVATE ${NSL_LIB_PATH} ${OpenCV_LIBS} pthread)
target_include_directories(nslApp PRIVATE ${NSL_LIB_INCLUDE_DIR})
endif()

