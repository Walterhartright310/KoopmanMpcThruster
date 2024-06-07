cmake_minimum_required(VERSION 3.8)
project(ros2gzbridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Added for quaternion
find_package(tf2 REQUIRED)           # Added for quaternion
find_package(tf2_geometry_msgs REQUIRED)  # Added for quaternion

 
 add_executable(odometry_subscriber src/tethys_states.cpp)
ament_target_dependencies(odometry_subscriber rclcpp nav_msgs tf2 tf2_geometry_msgs std_msgs geometry_msgs)
  
  add_executable(data_collect_thruster src/data_collect_thruster.cpp)
ament_target_dependencies(data_collect_thruster rclcpp nav_msgs tf2 tf2_geometry_msgs std_msgs geometry_msgs)

  add_executable(Thruster_mpc src/Thruster_mpc.cpp)
ament_target_dependencies(Thruster_mpc rclcpp nav_msgs tf2 tf2_geometry_msgs std_msgs geometry_msgs)
target_link_libraries(Thruster_mpc /usr/local/MATLAB/R2023b/bin/glnxa64/libmat.so /usr/local/MATLAB/R2023b/bin/glnxa64/libmx.so)


  install(TARGETS
  odometry_subscriber
  data_collect_thruster
  Thruster_mpc
  DESTINATION lib/${PROJECT_NAME})
  
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
