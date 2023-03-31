#ifndef TEST_NODE_HPP
#define TEST_NODE_HPP

// ROS includes.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

// for tf2 transformations
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// px4 messages for ros2
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

// for topic synch
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

using namespace std;
using namespace std::placeholders;
using std::placeholders::_2;


std::shared_ptr<rclcpp::Node> node = nullptr;

class TestNode : public rclcpp::Node
{
public:
  //! Constructor.
  TestNode();

  //! Destructor.
  ~TestNode();

private:

  std::string odom_source{"/fmu/vehicle_odometry/out"};
  std::string imu_source {"/fmu/vehicle_imu/out"};
  std::string lla_source {"/fmu/vehicle_global_position/out"};
  std::string pub_source;

  // parameters
  int _camera_frame_width{3840};
  int _camera_frame_height{2160};


  px4_msgs::msg::VehicleOdometry _odom_data;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _topic_publisher;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_subscriber;
  message_filters::Subscriber<px4_msgs::msg::VehicleImu> _imu_subscriber;
  message_filters::Subscriber<px4_msgs::msg::VehicleGlobalPosition> _global_position_subscriber;


  using approximate_policy = message_filters::sync_policies::ApproximateTime<px4_msgs::msg::VehicleImu, px4_msgs::msg::VehicleGlobalPosition>;
  typedef message_filters::Synchronizer<approximate_policy> Synchronizer;
  std::unique_ptr<Synchronizer> sync;

  // Callback function
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  // Publish messages
  void publishMessage();

  // Synchronized callback
  void syncedCallback(const px4_msgs::msg::VehicleImu::SharedPtr imuMsg, const px4_msgs::msg::VehicleGlobalPosition::SharedPtr globalPositionMsg);


  std::vector<double> quaternionToEuler(const geometry_msgs::msg::Quaternion& quat);

};

#endif