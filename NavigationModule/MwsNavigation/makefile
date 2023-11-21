INCLUDE_DIRECTORY = $(sort $(dir $(wildcard /opt/ros/iron/include/*/)))
LIB_FILE = $(sort $(wildcard /opt/ros/iron/lib/*.so))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRECTORY))
CXXLIB += $(addprefix ,$(LIB_FILE))

MwsNavigationController: MwsNavigation.cpp
	sudo g++ -fPIC -Wall -Wextra --std=c++17 MwsNavigation.cpp -O2 -o MwsNavigationController $(CXXFLAGS) -L /opt/ros/iron/lib -Wl,-rpath,/opt/ros/iron/lib $(CXXLIB) -lpthread -lconsole_bridge