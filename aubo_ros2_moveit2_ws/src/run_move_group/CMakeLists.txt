cmake_minimum_required(VERSION 3.5)
project(aubo_first_sim)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # build service file
# This shouldn't be necessary (required by moveit_simple_controller_manager)

add_executable(aubo_first_sim src/run_move_group.cpp)
ament_target_dependencies(aubo_first_sim
  moveit_ros_planning_interface
  rviz_visual_tools
)

install(TARGETS aubo_first_sim
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
