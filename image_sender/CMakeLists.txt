cmake_minimum_required(VERSION 3.5)
project(image_sender)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Build image sender
add_executable(image_sender src/image_sender.cpp)
ament_target_dependencies(image_sender rclcpp sensor_msgs OpenCV)

# Build image receiver
add_executable(image_receiver src/image_receiver.cpp)
ament_target_dependencies(image_receiver rclcpp sensor_msgs OpenCV)

# Install 
install(
  TARGETS image_sender image_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
