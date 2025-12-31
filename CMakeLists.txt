cmake_minimum_required(VERSION 3.21)
project(can_device)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_program(CCACHE_PROGRAM ccache)
if(NOT CCACHE_PROGRAM)
    message(STATUS "ccache not found")
else()
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_PROGRAM}")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  shared_types_nomad
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(shared_types_nomad REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# add_subdirectory(can_constants)


add_library(${PROJECT_NAME} 
  SHARED
  src/can_device.cpp
  can_constants/can_messages/output/can_messages.c
)



target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>

  # $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/can_constants/can_messages/output>
  # $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# target_include_directories(
#   ${PROJECT_NAME}
#   PUBLIC
#   can_constants/can_messages/output
# )

target_link_libraries(
  ${PROJECT_NAME}
  # can_constants
)

# ament_export_libraries(
#   can_constants
# )

ament_export_include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/can_constants/can_messages/output
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  shared_types_nomad
)

install(TARGETS 
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS can_device
  EXPORT export_can_device
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY can_constants/can_messages/output/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(rclcpp)


add_executable(can_device_node   src/can_node.cpp)
target_include_directories(can_device_node PUBLIC can_constants/can_messages/output)
ament_target_dependencies(can_device_node rclcpp )
target_link_libraries(can_device_node can_device)
install(TARGETS can_device_node DESTINATION lib/${PROJECT_NAME})


add_custom_target(generate_can_files
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/can_constants/./generate-files.sh -v
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_dependencies(${PROJECT_NAME} generate_can_files)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
