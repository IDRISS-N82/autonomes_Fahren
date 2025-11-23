cmake_minimum_required(VERSION 3.5)
project(psaf_firststeps)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(psaf_ucbridge_msgs REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneMarkings.msg"
  "msg/Trajectory.msg"
  "msg/State.msg"
  "msg/StatusInfo.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

set(NODE_NAME "firststeps")
add_executable(${NODE_NAME}
        src/${NODE_NAME}.cpp
        src/${NODE_NAME}_node.cpp)

target_include_directories(${NODE_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(${NODE_NAME}
        rclcpp
        std_msgs
        sensor_msgs
        psaf_ucbridge_msgs
        psaf_configuration)

add_executable(lane_detection_node src/lane_detection_node.cpp)
ament_target_dependencies(lane_detection_node
        rclcpp
        sensor_msgs
        geometry_msgs
        psaf_configuration)
target_include_directories(lane_detection_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(lane_detection_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(trajectory_node src/trajectory_node.cpp)
ament_target_dependencies(trajectory_node
        rclcpp
        geometry_msgs
        psaf_configuration)
target_include_directories(trajectory_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(trajectory_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
        rclcpp
        std_msgs
        geometry_msgs
        psaf_configuration)
target_include_directories(controller_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
        rclcpp
        std_msgs
        psaf_ucbridge_msgs
        psaf_configuration)
target_include_directories(state_machine_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(state_machine_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(watchdog_node src/watchdog_node.cpp)
ament_target_dependencies(watchdog_node
        rclcpp
        sensor_msgs
        std_msgs)
target_include_directories(watchdog_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(watchdog_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
        lane_detection_node trajectory_node controller_node state_machine_node watchdog_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(unit_tests test/unit_tests.cpp)

  install(TARGETS
          unit_tests
          DESTINATION lib/${PROJECT_NAME})

endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
