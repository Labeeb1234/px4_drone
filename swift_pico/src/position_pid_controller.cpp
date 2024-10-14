#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_array.hpp"
#include "swift_msgs/msg/swift_msgs.hpp"
#include "pid_msg/msg/pid_tune.hpp"
#include "pid_msg/msg/pid_error.hpp"
#include "rclcpp/rclcpp.hpp"


class PositionPIDController: public rclcpp::Node{
    public:
    PositionPIDController(): Node("position_pid_controller_node"){

    }
    ~PositionPIDController(){}
    
    void arming(){

    }


    private:

};



int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PositionPIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    
    return 0;
}