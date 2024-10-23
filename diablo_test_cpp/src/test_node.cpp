#include <iostream>
#include <unistd.h> // For usleep
#include <cmath>    // For math functions
#include "rclcpp/rclcpp.hpp"
#include "motion_msgs/msg/motion_ctrl.hpp"

using namespace std;
using namespace rclcpp;

class TestNode : public Node {
public:
    TestNode() : Node("test_node") {
        diablo_cmd = this->create_publisher<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 2);
        usleep(100000); // sleep for 0.1 seconds, allows time for publisher to establish connection
        performActions();
    }

private:
    Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr diablo_cmd;
    motion_msgs::msg::MotionCtrl ctrlMsgs;
    
    // When calling this function pass in the velocity you want to travel in meters per second.
    // To go in reverse pass in a negative value.
    // Robot will continue to move until you call move(0.0).
    void move(float velocity) {
        if (abs(velocity) <= 2.0) {
            ctrlMsgs.value.forward = velocity;
            diablo_cmd->publish(ctrlMsgs);            
        } else {
            cout << "Velocty value out of bounds" << endl;
        }
    }

    // When calling this function pass in the anglular velocity you want to turn with in radians per second.
    // To turn right pass in a negative value.
    // Robot will continue to turn until you call turn(0.0)
    void turn(float angularVelocity) {    
        if(abs(angularVelocity) <= 2.0) {
            ctrlMsgs.value.left = angularVelocity;
            diablo_cmd->publish(ctrlMsgs);   
        } else {
            cout << "Angular velocty value out of bounds" << endl;
        }     
    }
    
    // When calling this function pass in the height value for you robot. Value must be in [0,1.0]
    void stand(float height) {
        if (height >= 0.0 && height <= 1.0) {
            ctrlMsgs.mode_mark = true;
            ctrlMsgs.mode.stand_mode = true;
            diablo_cmd->publish(ctrlMsgs);
            ctrlMsgs.value.up = height;
            diablo_cmd->publish(ctrlMsgs);     
            ctrlMsgs.mode_mark = false;   
            diablo_cmd->publish(ctrlMsgs);      
            usleep(100); // Sleep for 0.0001 seconds for time to publish  
        } else {
            cout << "Height value out of bounds." << endl;
        }
    }
    
    // Function to enter creeping mode / sit down.
    void sit() {
        ctrlMsgs.mode_mark = true;
        ctrlMsgs.mode.stand_mode = false;
        diablo_cmd->publish(ctrlMsgs);
        ctrlMsgs.mode_mark = false; 
        diablo_cmd->publish(ctrlMsgs);    
        usleep(100); // Sleep for 0.0001 seconds for time to publish    
    }
    
    // When calling this function pass in the distance you want to travel in meters.
    // To go in reverse pass in a negative value.
    void moveDistance(float distance) {
        move(0.5);
        usleep(abs(distance) * 2000000); // sleep for distance * 2 seconds
        move(0.0);
        usleep(500000); // sleep for 0.5 seconds
    }

    // When calling this function pass in the angle you want to turn in radians.
    // To turn right pass in a negative value.
    void turnAtAngle(float angle) {
        turn((angle > 0) ? M_PI / 4 : -M_PI / 4);
        usleep(abs(angle) / (M_PI / 4) * 1000000); // sleep for angle / (pi/4) seconds
        turn(0.0);
        usleep(500000); // sleep for 0.5 seconds
    }

    // The method you will use to perform robot commands.
    void performActions() {   
        // Add control logic here
        
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


/*
// These are all the robot controls you will potentially use.
ctrlMsgs.mode_mark = false;
ctrlMsgs.mode.jump_mode = false;
ctrlMsgs.mode.split_mode = false;
ctrlMsgs.mode.height_ctrl_mode = false;
ctrlMsgs.mode.pitch_ctrl_mode = false;
ctrlMsgs.mode.stand_mode = false;
ctrlMsgs.mode.roll_ctrl_mode = false;        
ctrlMsgs.value.forward = 0.0; // value range: +-2.0 m/s, remark: negative numbers indicate backwards movement
ctrlMsgs.value.left = 0.0; // value range: +-2.0 rad/s, remark: negative numbers indicate right turn
ctrlMsgs.value.pitch = 0.0;
ctrlMsgs.value.roll = 0.0;
ctrlMsgs.value.up = 0.0;
        
diablo_cmd->publish(ctrlMsgs); // publishes control messages
*/
