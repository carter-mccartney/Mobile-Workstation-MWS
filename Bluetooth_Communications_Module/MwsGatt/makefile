INCLUDE_DIRECTORY = $(sort $(dir $(wildcard /opt/ros/iron/include/*/)))
LIB_FILE = $(sort $(wildcard /opt/ros/iron/lib/*.so))
CXXFLAGS += $(addprefix -I,$(INCLUDE_DIRECTORY))
CXXLIB += $(addprefix ,$(LIB_FILE))

MwsGatt: MwsGatt.cpp
	sudo g++ -fPIC -Wall -Wextra --std=c++17 MwsGatt.cpp -g -O2 -o MwsGatt /opt/gobbledegook/src/libggk.a -lgobject-2.0 -lgio-2.0 -lglib-2.0 -lpthread -I /opt/gobbledegook//include/ $(CXXFLAGS) -L /opt/ros/iron/lib -Wl,-rpath,/opt/ros/iron/lib $(CXXLIB)
	sudo setcap 'cap_net_raw,cap_net_admin+eip' ./MwsGatt