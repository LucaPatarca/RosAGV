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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#range node
add_executable(range src/nodes/range.cpp src/controller/range_sensor.cpp)
target_include_directories(range PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(range rclcpp sensor_msgs)

install(TARGETS range
  DESTINATION lib/${PROJECT_NAME})

#movement node
add_executable(movement src/nodes/movement.cpp src/controller/motor_controller.cpp)
target_include_directories(movement PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(movement rclcpp sensor_msgs)

install(TARGETS movement
  DESTINATION lib/${PROJECT_NAME})

#controller node
add_executable(controller src/nodes/controller.cpp)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(controller rclcpp sensor_msgs)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
find_package(WiringPi REQUIRED)
find_package(Threads REQUIRED)
include_directories(${WIRINGPI_INCLUDE_DIRS})
target_link_libraries(range ${WIRINGPI_LIBRARIES}) 
target_link_libraries(range ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(movement ${WIRINGPI_LIBRARIES}) 
target_link_libraries(movement ${CMAKE_THREAD_LIBS_INIT})