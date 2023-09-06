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

find_package(aaveq_ros_interfaces REQUIRED)

############################################################################

# External Libraries

include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master 
  )

FetchContent_Declare(sfml
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)

FetchContent_MakeAvailable(eigen sfml)
  
############################################################################

# Libraries

include_directories(include)

install(
  DIRECTORY include/usv_sim_2d
  DESTINATION include)

add_library(usv src/usv.cpp)
target_link_libraries(usv eigen)
ament_export_targets(usv HAS_LIBRARY_TARGET)

install(
  TARGETS usv
  EXPORT usv
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

############################################################################

# Nodes (C++)

# sim
add_executable(sim nodes/sim.cpp)
ament_target_dependencies(sim rclcpp std_msgs geometry_msgs aaveq_ros_interfaces)
target_link_libraries(sim usv sfml-graphics)


install(TARGETS
  sim
DESTINATION lib/${PROJECT_NAME})

############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

