gnome-terminal -x bash -c "~/Startup/Scripts/rsp.sh; exec bash"
sleep 20
gnome-terminal -x bash -c "~/Startup/Scripts/transform_publisher_laser.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/transform_publisher_imu.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/control.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/navigation.sh; exec bash"
sleep 20
gnome-terminal -x bash -c "~/Startup/Programs/Esp32Node; exec bash"
gnome-terminal -x bash -c "~/Startup/Programs/MwsNavigationController; exec bash"
sleep 60
gnome-terminal -x bash -c "~/Startup/Programs/MwsGatt; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/lidar.sh; exec bash"
