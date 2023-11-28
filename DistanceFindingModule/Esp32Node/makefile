INCLUDE_DIRECTORY = $(sort $(dir $(wildcard /opt/ros/iron/include/*/)))
LIB_FILE = $(sort $(wildcard /opt/ros/iron/lib/*.so))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRECTORY))
CXXLIB += $(addprefix ,$(LIB_FILE))

Esp32Node: EspNode.cpp commands.h Mapping.h Esp.h
	sudo g++ -fPIC -Wall -Wextra --std=c++17 -O2 -o ~/Startup/Programs/Esp32Node EspNode.cpp commands.h Mapping.h Esp.h ${CXXFLAGS} -L /opt/ros/iron/lib -Wl,-rpath,/opt/ros/iron/lib $(CXXLIB) -lexample_interfaces__rosidl_generator_c -lconsole_bridge -lserial -lstdc++fs -lpthread -lorocos-kdl -lkdl_parser