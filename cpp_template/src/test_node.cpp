#include <cpp_template/test_node.hpp>

/*--------------------------------------------------------------------
 * TestNode()
 * Constructor.
 *------------------------------------------------------------------*/
TestNode::TestNode():
Node("camera_raw_los_publisher")
// sync(MySyncPolicy(10), _imu_subscriber, _bounding_box_subscriber)
{ 
  // all params to be used
  this->declare_parameter("sensor_height_px", 1080);
  this->declare_parameter("sensor_width_px", 1920);
  this->declare_parameter("odom_source", "/fmu/vehicle_odometry/out");
  this->declare_parameter("pub_source", "");

  rclcpp::Parameter _pub_source_temp;
  rclcpp::Parameter odom_source_temp;
  rclcpp::Parameter _camera_frame_width_temp;
  rclcpp::Parameter _camera_frame_height_temp;


   if(!(this->get_parameter("sensor_height_px", _camera_frame_height_temp) &&
        this->get_parameter("sensor_width_px", _camera_frame_width_temp)))
    {
      RCLCPP_INFO(this->get_logger(), "Problem while loading parameters.");
      rclcpp::shutdown();
    }

  this->get_parameter("odom_source", odom_source_temp);
  this->get_parameter("pub_source", _pub_source_temp);

  _camera_frame_height = _camera_frame_height_temp.as_int();
  _camera_frame_width = _camera_frame_width_temp.as_int();
  odom_source = odom_source_temp.as_string();
  pub_source = _pub_source_temp.as_string();

  // RCLCPP_INFO(this->get_logger(), std::to_string(_camera_frame_height).c_str());

  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

  // subscribers
  _odom_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>
            (odom_source, sensor_qos, std::bind(&TestNode::odomCallback, this, _1));

  // publishers
  _topic_publisher = this->create_publisher<std_msgs::msg::float64>(pub_source, 10);

}


// Destructor.
TestNode::~TestNode()
{
}


// Publish the message.
void TestNode::publishMessage()
{

  std_msgs::msg::float64 _pub_msg;
  _pub_msg.data = _odom_data.vy;

  _topic_publisher->publish(_pub_msg);
}

void TestNode::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // in NED
  _odom_data = *msg;

  publishMessage();

}



int main(int argc, char **argv)
{

  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the node
  node = std::make_shared<TestNode>();

  rclcpp::spin(node);
  
}