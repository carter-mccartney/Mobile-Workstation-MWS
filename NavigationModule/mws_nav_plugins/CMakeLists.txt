cmake_minimum_required(VERSION 3.8)
project(mws_nav_plugins)

# Iterates through the list of dependancies and searches for them..
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
    include
  )

set(dependencies
  angles
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  nav2_core
  pluginlib
)

add_library(fake_goal_checker SHARED src/fake_goal_checker.cpp)
add_library(fake_progress_checker SHARED src/fake_progress_checker.cpp)
ament_target_dependencies(fake_goal_checker ${dependencies})
ament_target_dependencies(fake_progress_checker ${dependencies})
target_link_libraries(fake_goal_checker)
target_link_libraries(fake_progress_checker)

install(TARGETS fake_goal_checker fake_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(fake_goal_checker fake_progress_checker)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
# must be called exactly once.
# Installs the package.xml, registers the package with ament index, and installs config files so it may be found by other packages.
ament_package() 
