cmake_minimum_required(VERSION 3.8)
project(flexiv_moveit_codes_niwesh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)               # NEW: Required for quaternion math
find_package(tf2_geometry_msgs REQUIRED) # NEW: Required for pose orientation

# 2. Add the executable (ensure your cpp file is named flexiv_pick_place.cpp)
add_executable(flexiv_pick_place src/flexiv_pick_place.cpp)

target_include_directories(flexiv_pick_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(flexiv_pick_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# 3. Link the dependencies to the executable
ament_target_dependencies(
  flexiv_pick_place
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

# 4. Install the executable so `ros2 run` can find it
install(TARGETS flexiv_pick_place
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

ament_package()