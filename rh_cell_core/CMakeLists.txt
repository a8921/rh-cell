cmake_minimum_required(VERSION 3.16)
project(rh_cell_core)
find_package(ament_cmake REQUIRED)

# Plain C++17 library. Deliberately NO ROS includes: this code must run anywhere.
add_library(rh_cell_core src/admittance_1d.cpp src/force_guard.cpp)
target_compile_features(rh_cell_core PUBLIC cxx_std_17)
target_compile_options(rh_cell_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rh_cell_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(contact_demo src/contact_demo.cpp)
target_link_libraries(contact_demo rh_cell_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rh_cell_core EXPORT export_rh_cell_core
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS contact_demo DESTINATION lib/${PROJECT_NAME})
ament_export_targets(export_rh_cell_core HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core rh_cell_core)
endif()

ament_package()
