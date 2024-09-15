cmake_minimum_required(VERSION 3.5)
project(camera_opencv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
  include
)

add_executable(camera src/camera.cpp)
ament_target_dependencies(camera rclcpp sensor_msgs cv_bridge OpenCV image_transport)

install(TARGETS camera
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment the following line if you want to skip linter checks
  # set(ament_cmake_copyright_FOUND TRUE)
  # Uncomment the following line if you want to skip cpplint checks
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

