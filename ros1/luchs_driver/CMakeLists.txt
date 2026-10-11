cmake_minimum_required(VERSION 3.10)
project(luchs_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS roscpp luchs_msgs geometry_msgs nav_msgs sensor_msgs
  std_srvs tf2 tf2_ros diagnostic_updater)

# luchs_sdk is a plain CMake package: build the workspace with catkin build or
# catkin_make_isolated (plain catkin_make does not build non-catkin packages).
find_package(luchs_sdk 1.1 REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp luchs_msgs geometry_msgs nav_msgs sensor_msgs std_srvs
  tf2_ros diagnostic_updater)

add_executable(luchs_driver_node src/luchs_driver_node.cpp)
target_include_directories(luchs_driver_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(luchs_driver_node luchs_sdk::luchs_sdk ${catkin_LIBRARIES})
add_dependencies(luchs_driver_node ${catkin_EXPORTED_TARGETS})

install(TARGETS luchs_driver_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/dry_run_b.test)
  add_rostest(test/dry_run_a.test)
endif()
