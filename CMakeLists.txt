cmake_minimum_required(VERSION 3.8)
project(lidar_pcd_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${PCL_INCLUDE_DIRS}
        include
)

add_executable(pointcloud_saver_service src/pcl_saver.cpp)
target_link_libraries(pointcloud_saver_service ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_saver_service 
  rclcpp 
  sensor_msgs 
  std_srvs 
  PCL
  pcl_conversions
  )


add_executable(my_node src/main.cpp)
target_link_libraries(my_node ${PCL_LIBRARIES})

ament_target_dependencies(my_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  PCL
  tf2_ros
  std_srvs
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

install(TARGETS
  my_node
  pointcloud_saver_service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  urdf 
  worlds
  models 
  config 
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
