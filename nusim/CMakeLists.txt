cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv "srv/Teleport.srv" LIBRARY_NAME ${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(nusim src/nusim.cpp)
ament_target_dependencies(nusim
                          rclcpp
                          std_msgs
                          std_srvs
                          tf2
                          tf2_ros
                          turtlesim
                          visualization_msgs
                          nuturtlebot_msgs
                          turtlelib
                          nav_msgs)

target_link_libraries(nusim turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(basic_sensor src/basic_sensor.cpp)
ament_target_dependencies(basic_sensor
                          rclcpp
                          std_msgs
                          tf2
                          tf2_ros
                          visualization_msgs
                          nav_msgs
                          turtlelib)

target_link_libraries(basic_sensor turtlelib::turtlelib "${cpp_typesupport_target}")

add_executable(fake_laser_scan src/fake_laser_scan.cpp)
ament_target_dependencies(fake_laser_scan
                          rclcpp
                          std_msgs
                          tf2
                          tf2_ros
                          visualization_msgs
                          nav_msgs
                          sensor_msgs
                          turtlelib)

target_link_libraries(fake_laser_scan turtlelib::turtlelib "${cpp_typesupport_target}")

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

install(TARGETS
  nusim
  basic_sensor
  fake_laser_scan
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
