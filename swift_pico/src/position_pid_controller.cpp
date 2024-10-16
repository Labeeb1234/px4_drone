#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "pid_msg/msg/pid_tune.hpp"
#include "pid_msg/msg/pid_error.hpp"
#include "rclcpp/rclcpp.hpp"


// cmd value range [1000, 2000]
struct CMD
{
    int rc_roll;
    int rc_pitch;
    int rc_yaw;
    int rc_throttle;
    int rc_aux4;
};

struct ERROR
{
    float roll_error;
    float pitch_error;
    float throttle_error;
    float yaw_error;

};



class PositionPIDController: public rclcpp::Node{
    public:
    PositionPIDController(): Node("position_pid_controller_node"){
        cmd.rc_roll = 1500;
        cmd.rc_pitch = 1500;
        cmd.rc_yaw = 1500;
        cmd.rc_throttle = 1500;

        setpoint.resize(3);
        current_position.resize(3);
        setpoint = {0.1, 0.0, 19.0};
        current_position = {0.0, 0.0, 0.0};

        error.resize(3);
        prev_error.resize(3);
        interror.resize(3);
        Kp.resize(3);
        Kd.resize(3);
        Ki.resize(3);
        error = {0.0, 0.0, 0.0};
        interror = {0.0, 0.0, 0.0};
        prev_error = {0.0, 0.0, 0.0};


        // roll, pitch and throttle pid gain initialization
        Kp[0] = 0.0;
        Kp[1] = 0.0;
        Kp[2] = 0.0;

        Kd[0] = 0.0;
        Kd[1] = 0.0;
        Kd[2] = 0.0;

        Ki[0] = 0.0;
        Ki[1] = 0.0;
        Ki[2] = 0.0;

        swift_pub_ = this->create_publisher<swift_msgs::msg::SwiftMsgs>("drone_command", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("whycon/poses", 10, std::bind(&PositionPIDController::pose_callback, this, std::placeholders::_1));
        roll_pid_sub_ = this->create_subscription<pid_msg::msg::PIDTune>("roll_pid", 10, std::bind(&PositionPIDController::roll_set_pid, this, std::placeholders::_1));
        pitch_pid_sub_ = this->create_subscription<pid_msg::msg::PIDTune>("pitch_pid", 10, std::bind(&PositionPIDController::pitch_set_pid, this, std::placeholders::_1));
        throttle_pid_sub_ = this->create_subscription<pid_msg::msg::PIDTune>("throttle_pid", 10, std::bind(&PositionPIDController::throttle_set_pid, this, std::placeholders::_1));
        pid_error_pub_ = this->create_publisher<pid_msg::msg::PIDError>("pid_error", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<uint64_t>(this->control_time)), 
            std::bind(&PositionPIDController::control_loop, this)
        );

        this->arming(); // arming the drone during the node construction


    }
    ~PositionPIDController(){
        this->disarming();
    }

    void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
        // had to do coordinate transformation from whycon frames to gazebo frames
        current_position[0] = -(msg->poses[0].position.x);
        current_position[1] = -(msg->poses[0].position.y);
        current_position[2] = 32-(msg->poses[0].position.z);
    }

    void roll_set_pid(const pid_msg::msg::PIDTune& roll){
        Kp[0] = roll.kp;
        Kd[0] = roll.kd;
        Ki[0] = roll.ki;

    }
    void pitch_set_pid(const pid_msg::msg::PIDTune& pitch){
        Kp[1] = pitch.kp;
        Kd[1] = pitch.kd;
        Ki[1] = pitch.ki;
    }
    void throttle_set_pid(const pid_msg::msg::PIDTune& throttle){
        Kp[2] = throttle.kp;
        Kd[2] = throttle.kd;
        Ki[2] = throttle.ki;
    }
    
    void control_loop(){
        swift_msgs::msg::SwiftMsgs cmds;
        pid_msg::msg::PIDError error_msg;

        // calculating the axis errors
        error[0] = setpoint[0]-current_position[0];
        error[1] = setpoint[1]-current_position[1];
        error[2] = setpoint[2]-current_position[2];

        error_msg.roll_error = error[1];
        error_msg.pitch_error = error[0];
        error_msg.throttle_error = error[2];

        int roll_out = Kp[0]*(error[1])+(Kd[0]/0.06)*(error[1]-prev_error[1]) + Ki[0]*interror[0];
        int pitch_out = Kp[1]*(error[0])+(Kd[1]/0.06)*(error[0]-prev_error[0]) + Ki[1]*interror[1];
        int throttle_out = Kp[2]*(error[2])+(Kd[2]/0.06)*(error[2]-prev_error[2]) + Ki[2]*interror[2];

        prev_error[0] = error[0];
        prev_error[1] = error[1];
        prev_error[2] = error[2];

        interror[0] += error[0]*0.06;
        interror[1] += error[1]*0.06;
        interror[2] += error[2]*0.06;

        RCLCPP_INFO(
            this->get_logger(), "[x: %f, y: %f, z: %f]", 
            current_position[0], current_position[1], current_position[2]
        );

        cmds.rc_roll = 1500+roll_out;
        cmds.rc_pitch = 1500+pitch_out;
        cmds.rc_throttle = 1500+throttle_out;
        
        if(cmds.rc_roll > 2000){
            cmds.rc_roll = 2000;
        }

        if(cmds.rc_pitch > 2000){
            cmds.rc_pitch = 2000;
        }
        else if(cmds.rc_pitch < 1500){
            cmds.rc_pitch = 0;
        }

        if(cmds.rc_throttle > 2000){
            cmds.rc_throttle = 2000;
        }
        else if(cmds.rc_throttle < 1500){
            cmds.rc_throttle = 1500;
        }

        RCLCPP_INFO(this->get_logger(), "[Throttle: %ld, roll_rate: %ld, pitch_rate: %ld]", cmds.rc_throttle, cmds.rc_roll, cmds.rc_pitch);

        swift_pub_->publish(cmds);
        pid_error_pub_->publish(error_msg);

    }


    private: 
    rclcpp::Publisher<swift_msgs::msg::SwiftMsgs>::SharedPtr swift_pub_;
    rclcpp::Publisher<pid_msg::msg::PIDError>::SharedPtr pid_error_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<pid_msg::msg::PIDTune>::SharedPtr roll_pid_sub_, pitch_pid_sub_, throttle_pid_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    CMD cmd;
    float control_time = 60; // ms
    std::vector<double> setpoint, current_position;
    std::vector<double> error, prev_error, interror;
    std::vector<int> Kp, Ki, Kd; // roll, pitch, throttle

    bool arming(){

        swift_msgs::msg::SwiftMsgs msg;
        
        if(!disarming()){
            disarming();
        }
        
        // cmd values for arming (std values)
        msg.rc_roll = 0;
        msg.rc_pitch = 0;
        msg.rc_yaw = 0;
        msg.rc_throttle = 1500;
        msg.rc_aux4 = 1500;

        if((msg.rc_throttle < 1500) || (msg.rc_aux4 < 1500)){
            return false;
        }
        this->swift_pub_->publish(msg);
        return true;


    }

    bool disarming(){
        swift_msgs::msg::SwiftMsgs msg;
        // cmd values for disarming (std values)
        msg.rc_roll = 1000;
        msg.rc_pitch = 1000;
        msg.rc_yaw = 1000;
        msg.rc_throttle = 1000;
        msg.rc_aux4 = 1000;

        if(msg.rc_throttle >= 1500){
            return false;
        }
        this->swift_pub_->publish(msg);
        return true;

    }

    void takeoff(){
        swift_msgs::msg::SwiftMsgs msg;
        // std values for taking off
        if(arming()){
            msg.rc_roll = 1500;
            msg.rc_pitch = 1500;
            msg.rc_yaw = 1500;
            msg.rc_throttle = 1550;
            msg.rc_aux4 = 1500;

            this->swift_pub_->publish(msg);
        }
    }

    int constrain(int& val, int max_val, int min_val){
        if(val >= max_val){
            val = max_val;
        }
        else if(val < min_val){
            val = min_val;
        }
        return val;
    }

};



int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PositionPIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    
    return 0;
}