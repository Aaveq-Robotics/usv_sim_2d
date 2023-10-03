cmake_minimum_required(VERSION 3.8)
project(usv_sim_2d)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 20)


############################################################################

# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SFML COMPONENTS graphics REQUIRED)
find_package(jsoncpp REQUIRED)

find_package(aaveq_ros_interfaces REQUIRED)

############################################################################

# Libraries

include_directories(include)

install(
  DIRECTORY include/usv_sim_2d
  DESTINATION include)

## actuator
add_library(actuator src/actuator.cpp)
target_link_libraries(actuator Eigen3::Eigen jsoncpp)
ament_export_targets(actuator HAS_LIBRARY_TARGET)

install(
  TARGETS actuator
  EXPORT actuator
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

## propeller
add_library(propeller src/propeller.cpp)
target_link_libraries(propeller actuator Eigen3::Eigen jsoncpp)
ament_export_targets(propeller HAS_LIBRARY_TARGET)

install(
  TARGETS propeller
  EXPORT propeller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

## usv
add_library(usv src/usv.cpp)
target_link_libraries(usv actuator propeller Eigen3::Eigen jsoncpp)
ament_export_targets(usv HAS_LIBRARY_TARGET)

install(
  TARGETS usv
  EXPORT usv
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

## diff_drive
# add_library(diff_drive src/diff_drive.cpp)
# target_link_libraries(diff_drive usv actuator Eigen3::Eigen jsoncpp)
# ament_export_targets(diff_drive HAS_LIBRARY_TARGET)

# install(
#   TARGETS diff_drive
#   EXPORT diff_drive
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include)

## visualisation
add_library(visualisation src/visualisation.cpp)
target_link_libraries(visualisation usv Eigen3::Eigen sfml-graphics)
ament_export_targets(visualisation HAS_LIBRARY_TARGET)

install(
  TARGETS visualisation
  EXPORT visualisation
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

############################################################################

# Nodes (C++)

# sim
add_executable(sim nodes/sim.cpp)
ament_target_dependencies(sim rclcpp std_msgs geometry_msgs aaveq_ros_interfaces)
target_link_libraries(sim usv visualisation)

# servo_test
add_executable(servo_test nodes/servo_test.cpp)
ament_target_dependencies(servo_test rclcpp aaveq_ros_interfaces)
target_link_libraries(servo_test usv visualisation)


install(TARGETS
  sim
  servo_test
DESTINATION lib/${PROJECT_NAME})

############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

