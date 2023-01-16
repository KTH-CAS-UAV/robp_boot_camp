cmake_minimum_required(VERSION 3.0.2)
project(robp_boot_camp_motors)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robp_msgs
  roscpp
)

find_package(Boost REQUIRED COMPONENTS random)

catkin_package(
  CATKIN_DEPENDS geometry_msgs robp_msgs roscpp
  INCLUDE_DIRS include
  LIBRARIES kobuki_motors
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_custom_target(
  copy_kobuki_motors_lib ALL
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/lib/libkobuki_motors.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libkobuki_motors.so
)

add_library(kobuki_motors SHARED IMPORTED)
set_target_properties(kobuki_motors PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(kobuki_motors PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libkobuki_motors.so)
add_dependencies(kobuki_motors copy_kobuki_motors_lib)

add_executable(${PROJECT_NAME}_node src/motors.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME motors_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} kobuki_motors)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Boost::random
  kobuki_motors
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)