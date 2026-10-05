cmake_minimum_required(VERSION 3.16)
project(tugger_nav LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tugger_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator REQUIRED)

# ROS-free cores (unit tested directly).
add_library(tugger_core STATIC src/grid.cpp src/sim_core.cpp src/field_core.cpp src/vda5050_core.cpp)
target_include_directories(tugger_core PUBLIC include)
target_link_libraries(tugger_core PUBLIC nlohmann_json::nlohmann_json nlohmann_json_schema_validator)
set_target_properties(tugger_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(warehouse_sim src/warehouse_sim_node.cpp)
target_link_libraries(warehouse_sim tugger_core rclcpp::rclcpp tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${rosgraph_msgs_TARGETS})

add_executable(protective_field_monitor src/protective_field_monitor_node.cpp)
target_link_libraries(protective_field_monitor tugger_core rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${tugger_msgs_TARGETS})

add_executable(vda5050_bridge src/vda5050_bridge_node.cpp)
target_link_libraries(vda5050_bridge tugger_core rclcpp::rclcpp rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp ${nav_msgs_TARGETS} ${std_msgs_TARGETS} ${nav2_msgs_TARGETS} ${tugger_msgs_TARGETS})

install(TARGETS warehouse_sim protective_field_monitor vda5050_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config maps behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vda5050/ DESTINATION share/${PROJECT_NAME}/vda5050)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_field test/test_field.cpp)
  target_link_libraries(test_field tugger_core)
  ament_add_gtest(test_sim test/test_sim.cpp)
  target_link_libraries(test_sim tugger_core)
  ament_add_gtest(test_vda5050 test/test_vda5050.cpp)
  target_link_libraries(test_vda5050 tugger_core)
  target_compile_definitions(test_vda5050 PRIVATE SCHEMA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vda5050")
  add_launch_test(test/test_order_to_finished.py TIMEOUT 240)
endif()

ament_package()
