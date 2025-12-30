cmake_minimum_required(VERSION 3.10)
project(NslApp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# option BUILD_PCL : default OFF
option(BUILD_PCL "Build as PCL library" OFF)

if (BUILD_PCL)
find_package(PCL 1.2 REQUIRED COMPONENTS common visualization)
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Threads REQUIRED)

find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS nsl_lib/include REQUIRED)
# OS별 라이브러리 경로 자동 선택
if(WIN32)
    message(STATUS "Target OS: Windows")
    find_library(NSL_LIB_PATH
        NAMES nanolib
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/lib/windows/shared
        REQUIRED
    )

elseif(APPLE)
    message(STATUS "Target OS: macOS")
    find_library(NSL_LIB_PATH
        NAMES nanolib
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/lib/mac-12.0.5/shared
        REQUIRED
    )
	add_compile_options(-Wno-deprecated-declarations)
elseif(UNIX)
    # 리눅스 (x86/aarch 구분 필요 시 CMAKE_SYSTEM_PROCESSOR 사용)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        message(STATUS "Target OS: Linux (aarch64|arm64)")
        find_library(NSL_LIB_PATH
            NAMES nanolib
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/lib/aarch-7.5/shared
            REQUIRED
        )
    else()
        message(STATUS "Target OS: Linux (x86_64)")
        find_library(NSL_LIB_PATH
            NAMES nanolib
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/lib/linux-7.5/shared
            REQUIRED
        )
    endif()

	add_compile_options(-Wno-deprecated-declarations)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/mac-12.0.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/linux-7.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/aarch-7.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/windows/shared REQUIRED)


add_executable(nslApp main.cpp)

if (BUILD_PCL)
target_link_libraries(nslApp PRIVATE ${NSL_LIB_PATH} ${OpenCV_LIBS} ${PCL_LIBRARIES} Threads::Threads)
target_include_directories(nslApp PRIVATE ${NSL_LIB_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
else()
target_link_libraries(nslApp PRIVATE ${NSL_LIB_PATH} ${OpenCV_LIBS} Threads::Threads)
target_include_directories(nslApp PRIVATE ${NSL_LIB_INCLUDE_DIR})
endif()

if (WIN32)
	set(NSL_DLL_PATH
        ${CMAKE_CURRENT_SOURCE_DIR}/nsl_lib/lib/windows/shared/nanolib.dll
    )

    add_custom_command(TARGET nslApp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${NSL_DLL_PATH}"
            $<TARGET_FILE_DIR:nslApp>
    )
endif()
