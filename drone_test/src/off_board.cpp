#include "ros/ros.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandTOL.h"

#include <iostream>
#include <string>

using namespace std;


class OffBoardNode{
    private:
    ros::NodeHandle nh_;
    mavros_msgs::State current_state;
    ros::Time last_updated_time;
    
    ros::ServiceClient arming_client_;
    ros::ServiceClient drone_TOL_client_;
    ros::ServiceClient drone_mode_client_;

    ros::Subscriber drone_state_;

    ros::WallTimer timer_;

    public:
    OffBoardNode(){
        this->drone_state_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &OffBoardNode::DroneState, this);

        drone_mode_client_ = this->nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        drone_TOL_client_ = this->nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

        timer_ = this->nh_.createWallTimer(ros::WallDuration(0.1), bind(&OffBoardNode::DroneMode, this));
    
        ROS_INFO("Starting Offboard Node...");
        this->last_updated_time = ros::Time::now();
        
    }
    

    ~OffBoardNode(){
     ROS_INFO("StateSubscriberNode destructor called");
    }

    void DroneState(const mavros_msgs::State::ConstPtr& msg){
        this->current_state = *msg;
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
        
        if(this->current_state.mode != mode_name){
            if(this->drone_mode_client_.call(mode_req) && mode_req.response.mode_sent){
                ROS_INFO("Enabling [%s] mode", mode_name.c_str());
                return true;
            }
            this->last_updated_time = ros::Time::now();
        }
        return false;

    }

    void PX4Takeoff(float altitude, string mode){
        mavros_msgs::CommandTOL TOL_req;
        TOL_req.request.min_pitch = 5.0;
        TOL_req.request.altitude = 10.0;

        if(mode == "AUTO.TAKEOFF"){
            if(this->drone_TOL_client_.call(TOL_req) && TOL_req.response.success){
                ROS_INFO("Taking off to altitude [%f]", altitude);
            }

        }
        else{
            ROS_INFO("Drone Mode is not TAKEOFF.., Please enable TAKEOFF mode , QUIT and try again");
        }
    }

    void DroneMode(){
        ros::Time current_loop_time = ros::Time::now(); 
        //cout << (current_loop_time-last_updated_time) << endl; # to ensure the timer period is as specifed in the wall timer function in the constructor
        cout << this->current_state.mode << endl;
        last_updated_time = current_loop_time;      

    }



};



int main(int argc, char** argv){
    ros::init(argc, argv, "off_board_node");
    ROS_INFO("ROS initialized");

    OffBoardNode* node = new OffBoardNode();
    bool ret = node->ArmDrone(argv[1]);
    if(ret && argv[2]){
        node->EnablePX4Mode(argv[2]);
    }
    else if(argv[1]=="DISARM"){
        delete node;
        ros::waitForShutdown();
    }
    // ROS_INFO("StateSubscriberNode instance created");    
    
    ros::spin();
    // ROS_INFO("Node spinning");


    return 0;
}