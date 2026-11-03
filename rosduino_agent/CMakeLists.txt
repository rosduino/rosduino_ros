# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(rosduino_agent VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

set(ROSDUINO_BUILD_TESTS OFF CACHE BOOL "Host tests are not part of the ROS package" FORCE)
set(ROSDUINO_BUILD_FUZZ OFF CACHE BOOL "Fuzz targets are not part of the ROS package" FORCE)
set(ROSDUINO_BUILD_VMCU OFF CACHE BOOL "vMCU is built by the host CMake project" FORCE)
set(ROSDUINO_BUILD_SERIAL_TCP OFF CACHE BOOL "serial-tcp is a host tool, not the ROS package" FORCE)
set(ROSDUINO_SANITIZE OFF CACHE BOOL "Sanitizers are for host-debug, not colcon" FORCE)
set(_tool_host "${CMAKE_CURRENT_SOURCE_DIR}/../../host/CMakeLists.txt")
set(_vendor_host "${CMAKE_CURRENT_SOURCE_DIR}/vendor/host/CMakeLists.txt")
if(EXISTS "${_tool_host}")
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../host" rosduino_host)
elseif(EXISTS "${_vendor_host}")
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/vendor/host" rosduino_host)
else()
  message(FATAL_ERROR
    "RosDuino host not found (../../host or vendor/host). From rosduino_devtools run: make libs")
endif()

add_executable(rosduino_agent
  src/main.cpp
  src/rclcpp_backend.cpp
  src/type_registry.cpp
  src/diagnostics.cpp
)
target_include_directories(rosduino_agent PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(rosduino_agent PRIVATE cxx_std_17)
target_compile_options(rosduino_agent PRIVATE -Wall -Wextra -Werror)
target_link_libraries(rosduino_agent PRIVATE
  rosduino_bridge
  rosduino_transport
  rclcpp::rclcpp
  rcpputils::rcpputils
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  ${diagnostic_msgs_TARGETS}
)

install(TARGETS rosduino_agent
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(example_interfaces REQUIRED)
  # Sources use SPDX-License-Identifier; ament_copyright wants the verbose Apache block.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_type_registry
    test/test_type_registry.cpp
    src/type_registry.cpp
  )
  if(TARGET test_type_registry)
    target_include_directories(test_type_registry PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    target_compile_features(test_type_registry PRIVATE cxx_std_17)
    target_compile_options(test_type_registry PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(test_type_registry
      rclcpp::rclcpp
      rcpputils::rcpputils
      rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
      ${example_interfaces_TARGETS}
    )
  endif()
endif()

ament_package()
