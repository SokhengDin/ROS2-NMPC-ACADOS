cmake_minimum_required(VERSION 3.8)
project(ros2_nmpc)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find required packages
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Link file system library
link_libraries(stdc++fs)

# Set Acados path
set(ACADOS_INSTALL_DIR "/home/eroxii/ocp_ws/acados" CACHE PATH "Path to acados installation")

# Set the path to generated acados code
set(ACADOS_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/differential_drive")

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/acados
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
  ${ACADOS_GENERATED_DIR}
  ${ACADOS_GENERATED_DIR}/differential_drive_model
  ${ACADOS_GENERATED_DIR}/differential_drive_cost
  ${Python3_INCLUDE_DIRS}
)

# Check if the Acados solver library exists
set(ACADOS_SOLVER_LIB "${ACADOS_GENERATED_DIR}/libacados_ocp_solver_differential_drive.so")
if(NOT EXISTS ${ACADOS_SOLVER_LIB})
  message(FATAL_ERROR "Acados solver library not found at ${ACADOS_SOLVER_LIB}. 
  Make sure to generate it before building by running your Acados model generation script.")
else()
  # Copy Acados solver library to build directory
  file(COPY ${ACADOS_SOLVER_LIB} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

# Create Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTarget.srv"
)

# Create the project library
add_library(${PROJECT_NAME}_lib
  src/nmpc_differential_drive.cpp
  src/acados_cpp_wrapper.cpp
  ${ACADOS_GENERATED_DIR}/acados_solver_differential_drive.c
)

# Link the project library with Acados and other dependencies
target_link_libraries(${PROJECT_NAME}_lib
  ${ACADOS_INSTALL_DIR}/lib/libacados.so
  ${ACADOS_INSTALL_DIR}/lib/libblasfeo.so
  ${ACADOS_INSTALL_DIR}/lib/libhpipm.so
  ${CMAKE_CURRENT_BINARY_DIR}/libacados_ocp_solver_differential_drive.so
  ${Python3_LIBRARIES}
  ${Python3_NumPy_LIBRARIES}
)

# Add ROS 2 dependencies to the library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Create the ROS 2 node executable
add_executable(nmpc_differential_drive_node src/nmpc_differential_drive_node.cpp)

target_link_libraries(nmpc_differential_drive_node
  ${PROJECT_NAME}_lib
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(nmpc_differential_drive_node "${cpp_typesupport_target}")

ament_target_dependencies(nmpc_differential_drive_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

# Set RPATH for the executable
set_target_properties(nmpc_differential_drive_node PROPERTIES
  INSTALL_RPATH "${CMAKE_CURRENT_BINARY_DIR}:${ACADOS_INSTALL_DIR}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install targets
install(TARGETS
  nmpc_differential_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()