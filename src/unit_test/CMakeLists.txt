cmake_minimum_required(VERSION 3.8)
project(unit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(serial REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(brake_server_node scripts/brake_server.cpp)
ament_target_dependencies(brake_server_node rclcpp amr_interfaces serial)

add_executable(multi_thread_demo scripts/multi_thread.cpp)
ament_target_dependencies(multi_thread_demo rclcpp example_interfaces)

install(TARGETS
  brake_server_node
  multi_thread_demo
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

ament_package()
