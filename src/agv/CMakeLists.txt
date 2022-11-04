cmake_minimum_required(VERSION 3.5)
project(agv)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(WiringPi REQUIRED)
find_package(Threads REQUIRED)

include_directories(include ${WIRINGPI_INCLUDE_DIRS})

#range node
add_executable(range src/nodes/range.cpp src/controller/range_sensor.cpp)
ament_target_dependencies(range 
  rclcpp
  sensor_msgs
)

#movement node
add_executable(movement src/nodes/movement.cpp src/controller/motor_controller.cpp)
ament_target_dependencies(movement 
  rclcpp
  sensor_msgs
)

#controller node
add_executable(controller src/nodes/controller.cpp)
ament_target_dependencies(controller 
  rclcpp 
  sensor_msgs
)

#clamp node
add_executable(clamp src/nodes/clamp.cpp src/controller/clamp_controller.cpp)
ament_target_dependencies(clamp 
  rclcpp 
  sensor_msgs
)

#camera node
add_executable(camera src/nodes/camera.cpp)
ament_target_dependencies(camera
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries( camera
  cv_bridge
  opencv_highgui
  opencv_videoio
)

#install executables
install(
  TARGETS clamp controller movement range camera
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport pluginlib rosidl_default_runtime rclcpp sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# add wiringPi
target_link_libraries(range ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(movement ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(clamp ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})