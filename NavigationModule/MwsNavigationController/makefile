INCLUDE_DIRECTORY = $(sort $(dir $(wildcard /opt/ros/iron/include/*/)))
LIB_FILE = $(sort $(wildcard /opt/ros/iron/lib/*.so))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRECTORY))
CXXLIB += $(addprefix ,$(LIB_FILE))

MwsNavigationController: MwsNavigationController.cpp
	sudo g++ -fPIC -Wall -Wextra --std=c++17 ./MwsNavigationController.cpp -O2 -o ~/Startup/Programs/MwsNavigationController -lpthread ${CXXFLAGS} -L /opt/ros/iron/lib -Wl,-rpath,/opt/ros/iron/lib $(CXXLIB) -lexample_interfaces__rosidl_generator_c -lconsole_bridge -lorocos-kdl -lkdl_parser