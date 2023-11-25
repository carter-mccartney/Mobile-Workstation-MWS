

cmake_minimum_required(VERSION 3.8)
project(mws_robot)

# Iterates through the list of dependancies and searches for them..
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 'Targets' are entities that cmake will create.
#target_compile_features(mws_robot PUBLIC cxx_std_17)


# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY params description launch
  DESTINATION share/${PROJECT_NAME}
)

# must be called exactly once.
# Installs the package.xml, registers the package with ament index, and installs config files so it may be found by other packages.
ament_package() 
