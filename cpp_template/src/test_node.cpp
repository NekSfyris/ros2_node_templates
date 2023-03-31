#include <cpp_template/test_node.hpp>

/*--------------------------------------------------------------------
 * TestNode()
 * Constructor.
 *------------------------------------------------------------------*/
TestNode::TestNode():
Node("test_node")
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

  // define quality of service: all messages that you want to receive must have the same
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 1;
  custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

  // subscribers
  _odom_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>
            (odom_source, sensor_qos, std::bind(&TestNode::odomCallback, this, _1));

  // publishers
  _topic_publisher = this->create_publisher<std_msgs::msg::Float64>(pub_source, 10);


  // synchronized subscribers
  _imu_subscriber.subscribe(this, imu_source, custom_qos_profile);
  _global_position_subscriber.subscribe(this, lla_source, custom_qos_profile);

  sync.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(10), _imu_subscriber, _global_position_subscriber));

  // register the approximate time callback
  sync->registerCallback(&TestNode::syncedCallback, this);

}


// Destructor.
TestNode::~TestNode()
{
}


// Publish the message.
void TestNode::publishMessage()
{

  std_msgs::msg::Float64 _pub_msg;
  _pub_msg.data = _odom_data.vy;

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", std::to_string(_camera_frame_height).c_str());

  _topic_publisher->publish(_pub_msg);
}

// Normal callback
void TestNode::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // in NED
  _odom_data = *msg;

  publishMessage();

}

// Synchronized callback
void TestNode::syncedCallback(const px4_msgs::msg::VehicleImu::SharedPtr imuMsg, const px4_msgs::msg::VehicleGlobalPosition::SharedPtr globalPositionMsg)
{
  RCLCPP_INFO(this->get_logger(), "Synch Callback called!");
}


// ----------
// ---MAIN---
// ----------
int main(int argc, char **argv)
{

  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the node
  node = std::make_shared<TestNode>();

  rclcpp::spin(node);
  
}