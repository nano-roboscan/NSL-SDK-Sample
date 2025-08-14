cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0074)
  # Warning: Policy CMP0074 is not set: find_package uses <PackageName>_ROOT variables.
  cmake_policy(SET CMP0074 NEW)
endif()


project(roboscan_nsl3130)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(cv_bridge)

find_package(image_transport REQUIRED)
find_package(nav_msgs)
find_package(std_msgs)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL REQUIRED COMPONENTS common)

# nanosystems library 
find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS nsl_lib/include REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/linux-7.5/shared REQUIRED)
find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/aarch-7.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/windows/shared REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
#add_definition($PCL_DEFINITIONS})

include_directories(include/ 
${PCL_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS} 
${OpenCV_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
${NSL_LIB_INCLUDE_DIR})



# Build
add_executable(roboscan_publish_node src/roboscan_publish_node.cpp)
                                
target_include_directories(roboscan_publish_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(roboscan_publish_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(roboscan_publish_node
  sensor_msgs
  rclcpp
  std_msgs
  cv_bridge
  pcl_conversions
  Eigen3
  OpenCV
  image_transport
)

target_link_libraries(roboscan_publish_node 
${PCL_LIBRARIES}
${Boost_SYSTEM_LIBRARY}
${Boost_LIBRARIES}
${NSL_LIB_PATH}
)

install(TARGETS roboscan_publish_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
set(CMAKE_BUILD_TYPE Release)

ament_export_include_directories(include) 
ament_package()
