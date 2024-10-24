#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl

class DiabloTest(Node):
    def __init__(self):
        super().__init__("diablo_test_node")
        self.diablo_cmd = self.create_publisher(MotionCtrl, "diablo/MotionCmd", 2)
        self.ctrlMsgs = MotionCtrl()

    # When calling this function pass in the velocity you want to travel in meters per second as a float data type.
    # To go in reverse pass in a negative value.
    # Robot will continue to move until you call move(0.0).
    def move(self, velocity):
        if abs(velocity) <= 2.0:
            self.ctrlMsgs.value.forward = velocity
            self.diablo_cmd.publish(self.ctrlMsgs)
        else:
            print("Velocty value out of bounds")
            
    # When calling this function pass in the anglular velocity you want to turn in radians per second as a float data type.
    # To turn right pass in a negative value.
    # Robot will continue to turn until you call turn(0.0)     
    def turn(self, angular_velocity):
        if abs(angular_velocity) <= 2.0:
            self.ctrlMsgs.value.left = angular_velocity
            self.diablo_cmd.publish(self.ctrlMsgs)
        else:
            print("Angular velocty value out of bounds")

    # When calling this function pass in the height value for you robot to stand at as a float data type.
    # Value must be contained in [0,1.0]
    def stand(self, height):
        if height >= 0.0 and height <= 1.0:
            self.ctrlMsgs.mode_mark = True
            self.ctrlMsgs.mode.stand_mode = True
            self.diablo_cmd.publish(self.ctrlMsgs)
            self.ctrlMsgs.value.up = height
            self.diablo_cmd.publish(self.ctrlMsgs)  
            self.ctrlMsgs.mode_mark = False
            self.diablo_cmd.publish(self.ctrlMsgs)
            time.sleep(0.0001) # sleep for 0.0001 seconds for time to publish

    # Function to enter creeping mode / sit down.
    def sit(self):
        self.ctrlMsgs.mode_mark = True
        self.ctrlMsgs.mode.stand_mode = False
        self.diablo_cmd.publish(self.ctrlMsgs)
        self.ctrlMsgs.mode_mark = False
        self.diablo_cmd.publish(self.ctrlMsgs)  
        time.sleep(0.0001) # Sleep for 0.0001 seconds for time to publish 

    # Functino to move the robot a specified distance. 
    # When calling this function pass in the distance you want to travel in meters as a float data type.
    # To go in reverse pass in a negative value for distance.
    def moveDistance(self, distance):
        self.move(0.5)
        time.sleep(abs(distance)*2) # sleep for distance * 2 seconds
        self.move(0.0)
        time.sleep(0.5)

    # Function to turn the robot at a specified angle.
    # When calling this function pass in the angle you want to turn in radians as a float data type.
    # To turn right pass in a negative value for the angle.
    def turnAtAngle(self, angle):
        self.turn(math.pi/4 if angle > 0 else -math.pi/4)
        time.sleep(abs(angle)/(math.pi/4)) # sleep for angle / (pi/4) seconds
        self.turn(0.0)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args) 
    diablo_test = DiabloTest()

    # code goes here

    time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

