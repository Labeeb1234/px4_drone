// a temporary code to create a dynamic transform between odom and base_link or base frame id link for the simulation purposes
// In the hardware implementation the local planner code should be brodcasting such transforms

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"


#include <Eigen/Dense>
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
    // rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // rmw_qos_profile_t qos; // lower layer dds-api implementation
    // tf broadcasting here is unique and not shared
    unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_data_;
    bool odom_received_ = false;
    const uint timer_period_ = 50; // let time unit be ms

    Eigen::Matrix3d R_ned_to_enu_;
    Eigen::Matrix3d aircraft_to_baselink_;  // FRD->FLU frames (another form of NED->ENU)


public:
    OdomTfBroadcaster(): Node("odom_tf_broadcaster_node"){
        auto qos = rclcpp::SensorDataQoS();
        aircraft_to_baselink_ <<
            1, 0, 0,
            0,-1, 0,
            0, 0,-1;
        R_ned_to_enu_ <<
            0, 1, 0,
            1, 0, 0,
            0, 0,-1;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("filtered/odometry", 10, odom_sub_callback);
        // odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, odom_sub_callback);
        timer_ = this->create_wall_timer(chrono::milliseconds(timer_period_), std::bind(&OdomTfBroadcaster::tf_broadcaster_loop, this));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(), "Starting odom frame to base frame TF!");
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

    // function<void(px4_msgs::msg::VehicleOdometry::SharedPtr)> odom_sub_callback = [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
    //     odom_data_.header.stamp = this->get_clock()->now();
    //     odom_data_.header.frame_id = "odom";
    //     odom_data_.pose.pose.position.x = msg->position[1];
    //     odom_data_.pose.pose.position.y = msg->position[0];
    //     odom_data_.pose.pose.position.z = -msg->position[2];

    //     Eigen::Quaterniond q_ned(
    //         msg->q[0],
    //         msg->q[1],
    //         msg->q[2],
    //         msg->q[3]
    //     );
    //     // q_NED to q_ENU
    //     Eigen::Matrix3d R = R_ned_to_enu_*q_ned*aircraft_to_baselink_;
    //     Eigen::Quaterniond q_enu(R);
    //     odom_data_.pose.pose.orientation.w = q_enu.w();
    //     odom_data_.pose.pose.orientation.x = q_enu.x();
    //     odom_data_.pose.pose.orientation.y = q_enu.y();
    //     odom_data_.pose.pose.orientation.z = q_enu.z();
        
    //     // covariance data population
    //     odom_data_.pose.covariance[0]  = msg->position_variance[1]; // x
    //     odom_data_.pose.covariance[7]  = msg->position_variance[0]; // y
    //     odom_data_.pose.covariance[14] = msg->position_variance[2]; // z
    //     odom_data_.pose.covariance[21] = msg->orientation_variance[1]; // roll
    //     odom_data_.pose.covariance[28] = msg->orientation_variance[0]; // pitch
    //     odom_data_.pose.covariance[35] = msg->orientation_variance[2]; // yaw
    //     odom_received_ = true;
    // };

    void tf_broadcaster_loop(){
        if(!odom_received_){
            RCLCPP_WARN(this->get_logger(), "odom data not available/waiting for odom data");
            return;
        }
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = odom_data_.header.stamp;
        tf_msg.header.frame_id = "odom";
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