cmake_minimum_required(VERSION 3.8)
project(execute_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(waypoint_msgs REQUIRED)


add_executable(waypoint_server src/waypoint_server.cpp)
ament_target_dependencies(waypoint_server rclcpp waypoint_msgs)




target_include_directories(waypoint_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(waypoint_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  waypoint_server
  "moveit_ros_planning_interface"
  "rclcpp"
)





install(DIRECTORY
worlds
models
DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})


install(
    PROGRAMS
    scripts/planner_controller.py
    DESTINATION lib/${PROJECT_NAME}
    )



install(TARGETS 
waypoint_server

  DESTINATION lib/${PROJECT_NAME})

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


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
