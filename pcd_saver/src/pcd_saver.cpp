#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <iostream>
#include <vector>
#include <memory>
#include <utility>
#include <functional>

class PointCloudToPCD: public rclcpp::Node{
private:
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};

std::string cloud_topic_;
std::string prefix_;
bool transforms_avail_;
bool binary_;
bool compressed_;

void writePCDFile(const std::string & filename, const pcl::PCLPointCloud2 & cloud)
{
  pcl::PCDWriter writer;
  if (binary_) {
    if (compressed_) {
      writer.writeBinaryCompressed(filename, cloud);
    } else {
      writer.writeBinary(filename, cloud);
    }
  } else {
    // Default precision is 8
    writer.writeASCII(filename, cloud);
  }
}

public:
PointCloudToPCD(const rclcpp::NodeOptions& options): Node("point_cloud_to_pcd", options), binary_(false), compressed_(false){

    this->declare_parameter<std::string>("prefix_name", "pcl_to_pcd");
    this->declare_parameter<std::string>("cloud_topic_name", "front_left/depth_pcl_camera");
    this->declare_parameter<bool>("transforms_present", false);
    this->get_parameter("prefix_name", prefix_);
    this->get_parameter("cloud_topic_name", cloud_topic_);
    this->get_parameter("transforms_present", transforms_avail_);

    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    .keep_last(10)
    .reliable();
    RCLCPP_INFO(this->get_logger(), "QoS Settings:");
    RCLCPP_INFO(this->get_logger(), "  Reliability: %s", 
        sensor_qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ? "RELIABLE" : "BEST_EFFORT");
    RCLCPP_INFO(this->get_logger(), "  Durability: %s", 
        sensor_qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "TRANSIENT_LOCAL" : "VOLATILE");
    RCLCPP_INFO(this->get_logger(), "  History: %s", 
        sensor_qos.get_rmw_qos_profile().history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ? "KEEP_LAST" : "KEEP_ALL");
    RCLCPP_INFO(this->get_logger(), "  Depth: %ld", sensor_qos.get_rmw_qos_profile().depth);
    

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, sensor_qos, std::bind(&PointCloudToPCD::cloud_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "starting point cloud to pcd converter!");
}


void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

    RCLCPP_INFO(this->get_logger(), "%s", this->cloud_topic_.c_str());


    if(cloud_msg->data.empty()){
        RCLCPP_WARN(this->get_logger(), "point cloud data topic is empty...");
    }

    pcl::PCLPointCloud2 pcl_pc2;
    std::stringstream ss;
    rclcpp::Time time_stamp = cloud_msg->header.stamp;
    std::string frame_id = cloud_msg->header.frame_id;

    if(!this->transforms_avail_){
        ss << this->prefix_ << time_stamp.seconds() << "."
           << std::setw(9) << std::setfill('0') << time_stamp.nanoseconds()
           << ".pcd";

        RCLCPP_INFO(this->get_logger(), "Writing to %s", ss.str().c_str());
        pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    }
    // else{
    //     std::stringstream ss;
    //     ss << this->cloud_topic_ << time_stamp.seconds() << "."
    //        << std::setw(9) << std::setfill('0') << time_stamp.nanoseconds()
    //        << ".pcd";

    //     RCLCPP_INFO(this->get_logger(), "Writing to %s", ss.str().c_str());



    // }

    this->writePCDFile(ss.str(), pcl_pc2);

}

};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToPCD>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudToPCD);