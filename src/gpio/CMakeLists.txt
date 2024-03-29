cmake_minimum_required(VERSION 3.5)
project(gpio)

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
find_package(sensor_msgs REQUIRED)
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

#clamp node
add_executable(clamp src/nodes/clamp.cpp src/controller/clamp_controller.cpp)
ament_target_dependencies(clamp 
  rclcpp 
  sensor_msgs
)

#led node
add_executable(led src/nodes/led.cpp)
ament_target_dependencies(led 
  rclcpp 
  std_msgs
)

#install executables
install(
  TARGETS clamp movement range led
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

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
target_link_libraries(led ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})