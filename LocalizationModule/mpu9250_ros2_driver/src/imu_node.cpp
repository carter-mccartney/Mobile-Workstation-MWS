// https://www.youtube.com/watch?v=GNjN51NvJ6s
// https://github.com/polyhobbyist/ros_qwiic_icm_20948/blob/master/src/main.cpp
// https://github.com/pcdangio/ros-driver_mpu9250


#include "imu_node.h"


imu_node::imu_node(std::shared_ptr<driver> driver) : Node("mpu9250_publisher")
{
  /* Store imu driver. */
  imu_node::imu_driver = driver;

  imu_node::declareParameters();
  
  /* Retrieve the parameters of the node. More about parameters here: https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/ */
  imu_node::frameId = get_parameter("frame_id").as_string();
  imu_node::topicImu = get_parameter("topicImu").as_string();
  imu_node::topicMag = get_parameter("topicMag").as_string(); 
  imu_node::periodMillis = (long)(get_parameter("periodMillis").as_int());

  // Create publisher
  imu_node::publisherImu = this->create_publisher<sensor_msgs::msg::Imu>(imu_node::topicImu, 10);
  imu_node::publisherMag = this->create_publisher<sensor_msgs::msg::MagneticField>(imu_node::topicMag, 10);

  std::function<void (driver::data)> f = std::bind(&imu_node::read_data_callback, this, std::placeholders::_1);
  imu_node::imu_driver->set_data_callback(f);

  unsigned int i2c_bus = 1;
  unsigned int i2c_address = 0x68;
  unsigned int interrupt_gpio_pin = 4;

  // Arguments for i2c_open: pigpio handle, Bus_id, I2C address, interrupt pin.
  // I2c address is 1101000 when pin AD0 is pulled low and 1101001 when pin AD0 is pulled high.
  imu_node::imu_driver->initialize(i2c_bus, i2c_address, interrupt_gpio_pin);

  imu_node::timer = this->create_wall_timer(std::chrono::duration<long, std::milli>(periodMillis), std::bind(&imu_node::data_callback, this));

  std::cout << "Driver successfully initialized" << std::endl;
}

void imu_node::deinitialize_driver()
{
  try
  {
    imu_node::imu_driver->deinitialize();
    std::cout << "Driver successfully deinitialized" << std::endl;
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

/* This function is called at the end of the read_data() function in the imu_driver.
 * The read_data() function grabs fresh data and stores it in a data structure.
 * The fresh data is made available in this function. Data variables are listed below:
 *   
 *  data.accel_x
 *  data.accel_y
 *  data.accel_z
 *
 *  data.temp
 *
 *  data.gyro_x
 *  data.gyro_y
 *  data.gyro_z
 *
 *  data.magneto_x
 *  data.magneto_y
 *  data.magneto_z
 */
void imu_node::read_data_callback(driver::data data)
{
  auto messageMag = sensor_msgs::msg::MagneticField();
  messageMag.header.frame_id = imu_node::frameId;
  messageMag.header.stamp = rclcpp::Clock().now();
  messageMag.magnetic_field.x = static_cast<double>(data.magneto_x) * 0.000001;
  messageMag.magnetic_field.y = static_cast<double>(data.magneto_y) * 0.000001;
  messageMag.magnetic_field.z = static_cast<double>(data.magneto_z) * 0.000001;
  imu_node::publisherMag->publish(messageMag);

  auto messageImu = sensor_msgs::msg::Imu();
  messageImu.header.frame_id = imu_node::frameId;
  messageImu.header.stamp = messageMag.header.stamp;
  messageImu.linear_acceleration.x = static_cast<double>(data.accel_x) * 9.80665;
  messageImu.linear_acceleration.y = static_cast<double>(data.accel_y) * 9.80665;
  messageImu.linear_acceleration.z = static_cast<double>(data.accel_z) * 9.80665;

  messageImu.angular_velocity.x = static_cast<double>(data.gyro_x) * M_PI / 180.0;
  messageImu.angular_velocity.y = static_cast<double>(data.gyro_y) * M_PI / 180.0;
  messageImu.angular_velocity.z = static_cast<double>(data.gyro_z) * M_PI / 180.0;
  imu_node::publisherImu->publish(messageImu);
}

/* This function is called in the frequency specified. */
void imu_node::data_callback()
{
  /* Tells the IMU driver to grab the accelerometer, gyroscope, and magnetometer data.
   * This When data reads are finished, the read_data_callback() function is called
   * with the fresh data made available in the read_data_callback() function.
   */
  imu_node::imu_driver->read_data();
}

/* These parameters are declared for the node and last the entire lifetime of the node. 
 * More about node parameters are found here: https://docs.ros.org/en/iron/Concepts/Basic/About-Parameters.html
 * Tutorial on node parameters can be found here: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
 */
void imu_node::declareParameters()
{
  this->declare_parameter<std::string>("topicImu", "/imu/data_raw");
  this->declare_parameter<std::string>("topicMag", "/imu/mag");
  this->declare_parameter<std::string>("frame_id", "imu");
  this->declare_parameter("periodMillis", rclcpp::PARAMETER_INTEGER);
//   this->declare_parameter<bool>("calibrate", true);
//   this->declare_parameter<int>("gyro_range", MPU9250Sensor::GyroRange::GYR_250_DEG_S);
//   this->declare_parameter<int>("accel_range", MPU9250Sensor::AccelRange::ACC_2_G);
//   this->declare_parameter<int>("dlpf_bandwidth", MPU9250Sensor::DlpfBandwidth::DLPF_260_HZ);
//   this->declare_parameter<double>("gyro_x_offset", 0.0);
//   this->declare_parameter<double>("gyro_y_offset", 0.0);
//   this->declare_parameter<double>("gyro_z_offset", 0.0);
//   this->declare_parameter<double>("accel_x_offset", 0.0);
//   this->declare_parameter<double>("accel_y_offset", 0.0);
//   this->declare_parameter<double>("accel_z_offset", 0.0);
}


int main(int argc, char* argv[])
{
  /* Initializes the node. */
  rclcpp::init(argc, argv);

  std::shared_ptr<driver> pImuDriver = std::make_shared<rpi_driver>();

  /* Create a node named 'mpu9250_publisher'. */
  auto node = std::make_shared<imu_node>(pImuDriver);

  /* The 'spin' function is a blocking operation and keeps the node running. 
   * The node checks for events that occur in the subscribed topics, and service calls. 
   * The node also binds the topics/services/actions to the callbacks that we defined in publisher
   * so we can may interact with other topics and nodes. The node is kept running until
   * a request to kill this node is received and will thus return from the 'spin' function.
   */
  rclcpp::spin(node);

  /* Deinitialize driver. */
  node->deinitialize_driver();

  /* Shutdown/deinitialize the node. */
  rclcpp::shutdown();
  return 0;
}