cmake_minimum_required(VERSION 3.8)
project(rospital_gui)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force new C++ ABI
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# Enable strict compiler warnings, but don't treat as errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set build type to Debug with symbols
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set Qt5 settings
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Specify UI search paths explicitly
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(yaml-cpp REQUIRED)

# Define source files
set(SOURCE_FILES
  src/main.cpp
  src/mainwindow.cpp
  src/map_page.cpp
  src/rviz_page.cpp
  src/task_page.cpp
  src/task1_page.cpp
  src/task2_page.cpp
)

# Define header files for moc
set(HEADER_FILES
  include/rospital_gui/mainwindow.h
  include/rospital_gui/map_page.h
  include/rospital_gui/rviz_page.h
  include/rospital_gui/task_page.h
  include/rospital_gui/task1_page.h
  include/rospital_gui/task2_page.h
)

# Define UI files
set(UI_FILES
  src/mainwindow.ui
)

# Define resource files
set(RESOURCE_FILES
  resources.qrc
)

# Generate UI headers
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# Add resources
qt5_add_resources(RESOURCES ${RESOURCE_FILES})

# Define executable
add_executable(rospital_gui_node ${SOURCE_FILES} ${HEADER_FILES} ${UI_HEADERS} ${RESOURCES})

# Include directories
target_include_directories(rospital_gui_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rospital_gui
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz_rendering_INCLUDE_DIRS}
  ${rviz_default_plugins_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(rospital_gui_node
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  yaml-cpp
)

# Add dependencies
ament_target_dependencies(rospital_gui_node
  rclcpp
  rclcpp_action
  tf2_ros
  nav2_msgs
  geometry_msgs
  visualization_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
)

# Install executable
install(TARGETS rospital_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/rospital_gui/
  DESTINATION include/${PROJECT_NAME}
)

# Install UI files
install(FILES ${UI_FILES}
  DESTINATION share/${PROJECT_NAME}/ui
)

# Install resource files
install(FILES ${RESOURCE_FILES}
  DESTINATION share/${PROJECT_NAME}
)

# Install icons directory
install(DIRECTORY icons/
  DESTINATION share/${PROJECT_NAME}/icons
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()