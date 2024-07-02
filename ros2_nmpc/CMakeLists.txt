cmake_minimum_required(VERSION 3.8)
project(ros2_nmpc)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

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
endif()

# Create Service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTarget.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Create the project library
add_library(${PROJECT_NAME}_lib
  src/nmpc_differential_drive.cpp
  src/acados_cpp_wrapper.cpp
  src/ekf_differential_drive.cpp
  ${ACADOS_GENERATED_DIR}/acados_solver_differential_drive.c
)

# Link the project library with Acados and other dependencies
target_link_libraries(${PROJECT_NAME}_lib
  ${ACADOS_INSTALL_DIR}/lib/libacados.so
  ${ACADOS_INSTALL_DIR}/lib/libblasfeo.so
  ${ACADOS_INSTALL_DIR}/lib/libhpipm.so
  ${ACADOS_SOLVER_LIB}
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
add_executable(ekf_differential_drive_node src/ekf_differential_drive_node.cpp)

# Link the node executable with the project library
target_link_libraries(nmpc_differential_drive_node
  ${PROJECT_NAME}_lib
)

# Add dependencies for the custom messages
rosidl_target_interfaces(nmpc_differential_drive_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Add ROS 2 dependencies to the node
ament_target_dependencies(nmpc_differential_drive_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(ekf_differential_drive_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

# Set RPATH for the executable
set_target_properties(nmpc_differential_drive_node PROPERTIES
  INSTALL_RPATH "$ORIGIN:${CMAKE_INSTALL_PREFIX}/lib:${ACADOS_INSTALL_DIR}/lib:${ACADOS_GENERATED_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install targets
install(TARGETS
  nmpc_differential_drive_node
  ekf_differential_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the Acados solver library
install(FILES ${ACADOS_SOLVER_LIB}
  DESTINATION lib
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