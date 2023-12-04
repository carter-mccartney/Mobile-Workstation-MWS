from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    imu_raw_data_node = Node(
    package="mpu9250_ros2_driver", 
    executable="mpu9250_ros2_driver",
    output="screen",
    parameters=[
            {"periodMillis": 200}
        ]
    )

    imu_filtered_data_node = Node(
    package="imu_complementary_filter", 
    executable="complementary_filter_node",
    output="screen",
    parameters=[
            {"gain_acc": 0.01}
        ]
    )

    nodes = [
        imu_raw_data_node,
        imu_filtered_data_node,

    ]

    return LaunchDescription(nodes)
    
    return ld