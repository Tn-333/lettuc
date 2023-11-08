cmake_minimum_required(VERSION 3.5)
project(sample_c_service)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xarm_msgs REQUIRED)

add_executable(c_service_sample src/c_service_sample.cpp)
ament_target_dependencies(c_service_sample rclcpp xarm_msgs)

add_executable(service_function_test src/c_service_test.cpp)
ament_target_dependencies(service_function_test rclcpp xarm_msgs)

install(TARGETS 
  c_service_sample
  service_function_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
