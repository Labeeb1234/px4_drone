#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <chrono>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;

class PicoFrameBroadcaster : public rclcpp::Node{
public:
  PicoFrameBroadcaster(): Node("pico_frame_broadcaster"){

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    motor_tf_broadcasters_.resize(4);
    for(int i=0; i<4; i++){
        motor_tf_broadcasters_[i] = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("rotors/odometry", 10, bind(&PicoFrameBroadcaster::handle_turtle_pose, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(chrono::milliseconds(20), bind(&PicoFrameBroadcaster::static_broadcaster, this));
  }
  ~PicoFrameBroadcaster(){}


  void handle_turtle_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg){
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "swift_pico/base_link";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  void static_broadcaster(){

    for(int i=0; i<4; i++){
        geometry_msgs::msg::TransformStamped motor_t;
        
    }

  }

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    vector<unique_ptr<tf2_ros::TransformBroadcaster>> motor_tf_broadcasters_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<PicoFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}