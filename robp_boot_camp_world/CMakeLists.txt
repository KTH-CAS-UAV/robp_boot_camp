cmake_minimum_required(VERSION 3.0.2)
project(robp_boot_camp_world)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS angles geometry_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/world.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME world_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
