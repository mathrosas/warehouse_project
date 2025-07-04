cmake_minimum_required(VERSION 3.8)
project(attach_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

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

add_executable(attach_service_server src/attach_service_server.cpp)
ament_target_dependencies(attach_service_server rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs tf2_ros tf2_geometry_msgs)

add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs tf2_ros tf2_geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(attach_service_server ${cpp_typesupport_target})
target_link_libraries(approach_service_server ${cpp_typesupport_target})

install(TARGETS
   attach_service_server
   approach_service_server
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
