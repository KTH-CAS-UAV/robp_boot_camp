cmake_minimum_required(VERSION 3.0.2)
project(robp_boot_camp_distance_sensor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  robp_boot_camp_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS random)

catkin_package(
  CATKIN_DEPENDS robp_boot_camp_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_custom_target(
  copy_distance_sensor_lib ALL
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/libdistance_sensor.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libdistance_sensor.so
)

add_library(distance_sensor SHARED IMPORTED)
set_target_properties(distance_sensor PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(distance_sensor PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libdistance_sensor.so)
add_dependencies(distance_sensor copy_distance_sensor_lib)

add_executable(${PROJECT_NAME}_node src/distance_sensor_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME distance_sensor_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} distance_sensor)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Boost::random
  distance_sensor
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)