cmake_minimum_required(VERSION 3.16)
project(luchs_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(luchs_sdk REQUIRED)
find_package(luchs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(luchs_driver_component SHARED src/luchs_driver_node.cpp)
target_include_directories(luchs_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(luchs_driver_component luchs_sdk::luchs_sdk)
ament_target_dependencies(luchs_driver_component
  luchs_msgs rclcpp rclcpp_components geometry_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_ros
  diagnostic_updater)
rclcpp_components_register_node(luchs_driver_component
  PLUGIN "luchs_driver::LuchsDriverNode"
  EXECUTABLE luchs_driver_node)

install(TARGETS luchs_driver_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_luchs_driver test/test_parameters.cpp)
  target_link_libraries(test_luchs_driver luchs_driver_component)
endif()

ament_package()
