cmake_minimum_required(VERSION 3.8)
project(usv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(lqr_control src/lqr_control.cpp)
ament_target_dependencies(
  lqr_control
  rclcpp 
  std_msgs 
  geometry_msgs 
  ros_gz_interfaces
  nav_msgs
  tf2
  tf2_geometry_msgs)

  add_executable(generate_ref src/generate_ref.cpp)
  ament_target_dependencies(
    generate_ref
    rclcpp
    std_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
  )
install(TARGETS 
  lqr_control
  generate_ref
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
