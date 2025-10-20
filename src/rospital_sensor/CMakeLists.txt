cmake_minimum_required(VERSION 3.8)
project(rospital_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm các package ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Tìm libserial bằng pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# Thêm include directories
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
)

# Target cho kalman_filter
add_executable(kalman_filter src/kalman_filter.cpp)
ament_target_dependencies(kalman_filter rclcpp nav_msgs sensor_msgs)

# Target cho imu_reader (dùng libserial)
add_executable(imu_reader src/imu_reader.cpp)
ament_target_dependencies(imu_reader rclcpp sensor_msgs)  # Chỉ các dependency ROS2
target_include_directories(imu_reader PUBLIC ${SERIAL_INCLUDE_DIRS})
target_link_libraries(imu_reader ${SERIAL_LIBRARIES})

# Target cho imu_republisher
add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(imu_republisher rclcpp sensor_msgs)

# Target cho odometry_motion_model
add_executable(odometry_motion_model src/odometry_motion_model.cpp)
ament_target_dependencies(odometry_motion_model rclcpp tf2 nav_msgs geometry_msgs)

# Cài đặt include
install(
  DIRECTORY include/
  DESTINATION include
)

# Cài đặt các target
install(TARGETS
  kalman_filter
  imu_reader
  imu_republisher
  odometry_motion_model
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()