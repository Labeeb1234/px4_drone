#include "ros/ros.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/PoseStamped.h"

#include <iostream>
#include <string>

using namespace std;


class OffBoardNode{
    private:
    ros::NodeHandle nh_;
    mavros_msgs::State current_state;
    float current_altitude;
    string drone_mode="";
    // bool mode_set = false;
    ros::Time last_updated_time;
    
    ros::ServiceClient arming_client_;
    ros::ServiceClient drone_TOL_client_;
    ros::ServiceClient drone_mode_client_;

    ros::Subscriber drone_state_;
    ros::Subscriber drone_altitude_sub_;

    ros::Publisher local_pose_pub_;

    ros::WallTimer timer_;
    bool timer_started_ = false;

    public:
    OffBoardNode(){
        local_pose_pub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        
        drone_state_ = this->nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &OffBoardNode::DroneState, this);
        drone_altitude_sub_ = this->nh_.subscribe<mavros_msgs::Altitude>("/mavros/altitude", 10, &OffBoardNode::DroneAltitudeCallback, this);

        drone_mode_client_ = this->nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        drone_TOL_client_ = this->nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
        
        //float offboard_freq_ = 5.0; // [Hz] 
        //timer_ = this->nh_.createWallTimer(ros::WallDuration(1/offboard_freq_), bind(&OffBoardNode::DroneMode, this));
    
        ROS_INFO("Starting Offboard Node...");
        this->last_updated_time = ros::Time::now();
        
    }
    

    ~OffBoardNode(){
     ROS_INFO("StateSubscriberNode destructor called");
    }

    void DroneState(const mavros_msgs::State::ConstPtr& msg){
        this->current_state = *msg;
    }

    void DroneAltitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg){
        auto alt_msg = *msg;
        this->current_altitude = alt_msg.relative;
    }

    bool ArmDrone(string arm){
        mavros_msgs::CommandBool arm_req;

        if(arm == "ARM"){
            arm_req.request.value = true;

            if(!this->current_state.armed){
                if(this->arming_client_.call(arm_req) && arm_req.response.success){
                    ROS_INFO("Arming Drone please standby...");
                    return true;
                }
            }
        }
        else if(arm == "DISARM"){
            arm_req.request.value = false;
            if(this->current_state.armed){
                if(this->arming_client_.call(arm_req) && arm_req.response.success){
                    ROS_INFO("Disarming Drone please standby...");
                    return false;
                }
            }
            return false;

        }
        return false;
    }


    bool EnablePX4Mode(string mode_name){
        mavros_msgs::SetMode mode_req;
        mode_req.request.custom_mode = mode_name;
        this->drone_mode = mode_name;
        
        if(this->current_state.mode != mode_name){
            if(this->drone_mode_client_.call(mode_req) && mode_req.response.mode_sent){
                ROS_INFO("Enabling [%s] mode", mode_name.c_str());
                this->StartTimer();
                // this->mode_set = true;
                return true;
            }
            this->last_updated_time = ros::Time::now();
        }
        return false;

    }

    void PX4Takeoff(string mode){
        mavros_msgs::CommandTOL TOL_req;
        TOL_req.request.min_pitch = 0.0;
        TOL_req.request.yaw = 2.0;
        TOL_req.request.altitude = 2.0;

        if(mode == "AUTO.TAKEOFF"){
            if(this->drone_TOL_client_.call(TOL_req) && TOL_req.response.success){
                ROS_INFO("Taking off to altitude [%f]", 2.0);
            }

        }
        else{
            ROS_INFO("Drone Mode is not TAKEOFF.., Please enable TAKEOFF mode , QUIT and try again");
        }
    }

    void DroneMode(){
        ros::Time current_loop_time = ros::Time::now(); 
        geometry_msgs::PoseStamped local_pose_msg;
        //cout << (current_loop_time-last_updated_time) << endl; # to ensure the timer period is as specifed in the wall timer function in the constructor
        //cout << this->current_state.mode << endl;
        ROS_INFO("Current Altitude: [%f]", this->current_altitude);
        
        local_pose_msg.header.stamp = ros::Time::now();
      
        local_pose_msg.pose.position.x = 0.0;
        local_pose_msg.pose.position.y = 0.0;
        local_pose_msg.pose.position.z = 10.0;

        this->local_pose_pub_.publish(local_pose_msg);





        last_updated_time = current_loop_time;      

    }

    void StartTimer(){
        if (!timer_started_){
            float offboard_freq_ = 5.0; // [Hz] 
            timer_ = nh_.createWallTimer(ros::WallDuration(1/offboard_freq_), bind(&OffBoardNode::DroneMode, this));
            timer_started_ = true;
        }
    }



};



int main(int argc, char** argv){
    ros::init(argc, argv, "off_board_node");
    ROS_INFO("ROS initialized");

    OffBoardNode* node = new OffBoardNode();
    
    // bool ret = node->ArmDrone(argv[1]);
    bool ret2 = node->EnablePX4Mode(argv[1]);  

    // if(ret && argv[2]){
    //     bool ret2 = node->EnablePX4Mode(argv[2]);       
    // }

    // ROS_INFO("StateSubscriberNode instance created");    
    
    ros::spin();
    // ROS_INFO("Node spinning");


    return 0;
}