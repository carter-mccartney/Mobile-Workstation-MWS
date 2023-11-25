# https://github.com/pcdangio/ros-driver_mpu9250/blob/master/CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(mpu9250_ros2_driver)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -lpigpiod_if2)
# endif()

add_compile_options(-Wall -Wextra -Wpedantic -lpigpiod_if2 -lrt)

# List of dependancies to find.
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
)

# Find pigpio for building rpi node.
find_library(pigpiod pigpiod_if2)

add_executable(${PROJECT_NAME} src/imu_node.cpp src/rpi_driver.cpp src/driver.cpp)

target_link_libraries(${PROJECT_NAME}
  ${pigpiod}
)

# Iterates through the list of dependancies and searches for them..
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 'Targets' are entities that cmake will create.
target_compile_features(mpu9250_ros2_driver PUBLIC cxx_std_17)


# find dependencies
find_package(ament_cmake REQUIRED)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "sensor_msgs"
)


install(DIRECTORY src/launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# must be called exactly once.
# Installs the package.xml, registers the package with ament index, and installs config files so it may be found by other packages.
ament_package() 
