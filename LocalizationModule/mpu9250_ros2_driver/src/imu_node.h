#ifndef IMU_NODE_H
#define IMU_NODE_H

#include <chrono>
#include <memory>
#include <math.h>
#include <stdexcept>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rpi_driver.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

/* Inherit basic ROS2 node functionality. */
class imu_node : public rclcpp::Node {
 public:
    // CONSTRUCTORS
    /// \brief ros_node Initializes the ROS node.
    /// \param driver The MPU9250 driver instance.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    imu_node(std::shared_ptr<driver> driver);

    // METHODS
    /// \brief deinitialize_driver Deinitializes the driver.
    void deinitialize_driver();
    /* This function is called in the frequency specified by the node. */
    void read_data_callback(driver::data data);

 private:
    // COMPONENTS
    /// \brief imu_driver The driver instance.
    std::shared_ptr<driver> imu_driver;
    /* Creates a shared pointer for the imu sensor msg publisher. */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisherImu;
    /* Creates a shared pointer for the magnetometer sensor msg publisher. */
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisherMag;
    /* Used for time stamping messages. */
    rclcpp::TimerBase::SharedPtr timer;
    /* The frame ID (position in the world) of the IMU sensor. Used for transforms. */
    std::string frameId;
    /* The topic for which this node publishes raw IMU (accelerometer and gyroscope) data to. */
    std::string topicImu;
    /* The topic for which this node publishes raw magnetometer data to. */
    std::string topicMag;
    /* Rate at which the data_callback function in this node is executed. */
    long periodMillis;

    int m_pigpio_handle;

    // CALIBRATIONS
    /// \brief The accelerometer's calibration.
    //calibration m_calibration_accelerometer;
    /// \brief The gyroscope's calibration.
    //calibration m_calibration_gyroscope;
    /// \brief The magnetometer's calibration.
    //calibration m_calibration_magnetometer;

    // METHODS
    /* Initializes node parameters. */
    void declareParameters();
    /* Calculates orienctation of the IMU. */
    void calculateOrientation(sensor_msgs::msg::Imu& imu_message);

    // CALLBACKS
    /// \brief data_callback The callback function for when new data is available from the imu.
    /// \param data The latest data read from the MPU9250/AK8963.
    void data_callback();
};

#endif  // IMU_NODE_H