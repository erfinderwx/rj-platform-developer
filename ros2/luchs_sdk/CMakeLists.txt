cmake_minimum_required(VERSION 3.16)
project(luchs_sdk VERSION 1.1.0 LANGUAGES CXX)

# Builds inside a colcon/ament workspace or stand-alone:
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
endif()

find_package(ament_cmake QUIET)
find_package(Threads REQUIRED)

add_library(luchs_sdk
  src/legacy_a_protocol.cpp
  src/lci_protocol.cpp
  src/legacy_b_protocol.cpp
  src/kinematics.cpp
  src/socketcan.cpp
  src/chassis.cpp
  src/sim_bus.cpp
  src/sim_bus_b.cpp
)
add_library(luchs_sdk::luchs_sdk ALIAS luchs_sdk)
target_include_directories(luchs_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(luchs_sdk PUBLIC Threads::Threads)
set_target_properties(luchs_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(luchs_cli apps/luchs_cli.cpp)
target_link_libraries(luchs_cli PRIVATE luchs_sdk)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS luchs_sdk EXPORT luchs_sdkTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
install(TARGETS luchs_cli RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS luchs_cli RUNTIME DESTINATION bin)
install(DIRECTORY examples/python/ DESTINATION share/${PROJECT_NAME}/examples/python
  USE_SOURCE_PERMISSIONS)

if(ament_cmake_FOUND)
  ament_export_targets(luchs_sdkTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Threads)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_luchs_sdk test/test_protocol.cpp test/test_kinematics.cpp test/test_chassis.cpp test/test_lci.cpp test/test_luchs_b.cpp)
    target_link_libraries(test_luchs_sdk luchs_sdk)
  endif()
  ament_package()
else()
  # find_package(luchs_sdk) for plain CMake and catkin (ROS 1) users
  install(EXPORT luchs_sdkTargets NAMESPACE luchs_sdk:: DESTINATION lib/cmake/luchs_sdk)
  include(CMakePackageConfigHelpers)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/luchs_sdkConfig.cmake
    "include(CMakeFindDependencyMacro)\nfind_dependency(Threads)\n"
    "include(\"\${CMAKE_CURRENT_LIST_DIR}/luchs_sdkTargets.cmake\")\n")
  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/luchs_sdkConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/luchs_sdkConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/luchs_sdkConfigVersion.cmake DESTINATION lib/cmake/luchs_sdk)

  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
      add_executable(test_luchs_sdk test/test_protocol.cpp test/test_kinematics.cpp test/test_chassis.cpp test/test_lci.cpp test/test_luchs_b.cpp)
      target_link_libraries(test_luchs_sdk luchs_sdk GTest::gtest_main)
      add_test(NAME test_luchs_sdk COMMAND test_luchs_sdk)
    endif()
  endif()
endif()
