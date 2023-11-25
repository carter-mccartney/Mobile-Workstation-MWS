gnome-terminal -x bash -c "~/Startup/Scripts/rsp.sh; exec bash"
sleep 10
gnome-terminal -x bash -c "~/Startup/Scripts/transform_publisher.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/slam.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/localization.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/control.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Scripts/navigation.sh; exec bash"
gnome-terminal -x bash -c "~/Startup/Programs/MwsNavigationController; exec bash"
sleep 45
gnome-terminal -x bash -c "~/Startup/Programs/MwsGatt; exec bash"
