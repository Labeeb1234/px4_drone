#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

using namespace std;

class PCRealsenseGenerator{
private:
ros::NodeHandle nh_;

ros::Publisher pc_pub_;
ros::Subscriber pc_sub_;

ros::WallTimer timer_;
sensor_msgs::PointCloud2 pc_data;


public:
PCRealsenseGenerator(){
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_generator/click_map", 10);
    pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/camera/depth/color/points", 1, &PCRealsenseGenerator::pc_data_callback, this
    );

    timer_ = nh_.createWallTimer(ros::WallDuration(0.1), bind(&PCRealsenseGenerator::map_data_timer, this));

};

~PCRealsenseGenerator(){};

void pc_data_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    this->pc_data = *msg;
}

void map_data_timer(){
    pc_pub_.publish(this->pc_data);
}


};


int main(int argc, char** argv){
    ros::init(argc, argv, "pc_realsense_generator");
    PCRealsenseGenerator node();
    ros::spin();
    ros::waitForShutdown();
    
    
    return 0;
}