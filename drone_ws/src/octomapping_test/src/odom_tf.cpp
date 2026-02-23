// a temporary code to create a dynamic transform between odom and base_link or base frame id link for the simulation purposes
// In the hardware implementation the local planner code should be brodcasting such transforms

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <iostream>
#include <cstdio>
#include <chrono>
// #include <thread>
#include <functional>
#include <memory>
#include <utility>

using namespace std;

// standard tf broadcaster node
class OdomTfBroadcaster: public rclcpp::Node{
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // rmw_qos_profile_t qos; // lower layer dds-api implementation
    // tf broadcasting here is unique and not shared
    unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_data_;
    bool odom_received_ = false;
    const uint timer_period_ = 100; // let time unit be ms

public:
    OdomTfBroadcaster(): Node("odom_tf_broadcaster_node"){
        rclcpp::QoS qos(10); // custom qos settings

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("filtered/odometry", 10, odom_sub_callback);
        timer_ = this->create_wall_timer(chrono::milliseconds(timer_period_), std::bind(&OdomTfBroadcaster::tf_broadcaster_loop, this));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    ~OdomTfBroadcaster(){}

    // member variable
    function<void(nav_msgs::msg::Odometry::SharedPtr)> odom_sub_callback = [this](nav_msgs::msg::Odometry::SharedPtr msg){
        odom_data_.header.stamp = msg->header.stamp;
        odom_data_.header.frame_id = msg->header.frame_id;
        odom_data_.pose.pose.position = msg->pose.pose.position;
        odom_data_.pose.pose.orientation = msg->pose.pose.orientation;
        odom_data_.pose.covariance = msg->pose.covariance;
        odom_received_ = true;
    };

    void tf_broadcaster_loop(){
        if(!odom_received_){
            RCLCPP_WARN(this->get_logger(), "odom data not available/waiting for odom data");
            return;
        }
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = odom_data_.header.stamp;
        tf_msg.header.frame_id = odom_data_.header.frame_id;
        tf_msg.child_frame_id = "x500_obs_0/base_footprint";
        tf_msg.transform.translation.x = odom_data_.pose.pose.position.x; 
        tf_msg.transform.translation.y = odom_data_.pose.pose.position.y; 
        tf_msg.transform.translation.z = odom_data_.pose.pose.position.z;
        tf_msg.transform.rotation.x = odom_data_.pose.pose.orientation.x;
        tf_msg.transform.rotation.y = odom_data_.pose.pose.orientation.y;
        tf_msg.transform.rotation.z = odom_data_.pose.pose.orientation.z;
        tf_msg.transform.rotation.w = odom_data_.pose.pose.orientation.w;
        
        this->tf_broadcaster_->sendTransform(tf_msg);
    }
};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTfBroadcaster>();
    try{
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }catch(const exception& e){
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}