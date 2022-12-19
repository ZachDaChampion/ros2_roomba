cmake_minimum_required(VERSION 3.8)
project(simple_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(create_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(simplecontroller src/simplecontroller.cpp)
ament_target_dependencies(simplecontroller rclcpp geometry_msgs create_msgs nav_msgs tf2_geometry_msgs)

target_include_directories(simplecontroller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simplecontroller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# add_executable(odom_printer src/odom_printer.cpp)
# ament_target_dependencies(odom_printer rclcpp geometry_msgs nav_msgs)

# target_include_directories(odom_printer PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(odom_printer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simplecontroller # odom_printer
  DESTINATION lib/${PROJECT_NAME})

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
