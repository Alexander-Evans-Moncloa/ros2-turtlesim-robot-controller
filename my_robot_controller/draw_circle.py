#!/usr/bin/env python3
import rclpy
from rclpy.node import Node #Because we are going to use the Node class
from geometry_msgs.msg import Twist
#~/ros2_ws$ colcon build --symlink-install after creating a new node and adding it to setup.py in the entry points


class DrawCircleNode(Node): #Creating this node

    def __init__(self): #Creates constructor
        super().__init__("draw_circle") #Calls constructor from upper class, gives node the name "draw_circle", can be anything.
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)   #We are making a publisher to allow us to send messages to a topic. "Self." is rclpy.node.
        #Above has Twist, the existing topic name (in turtlesim), /turtle1/cmd_vel (where the topic is located) and 10 (queue size, creates buffer of 10 messages)
        self.timer = self.create_timer(0.5, self.send_velocity_command) #Timer created when node created, calls function below every 0.5 seconds
        self.get_logger().info("Draw circle node has been started") #Print to console, useful to know when node has been started

    def send_velocity_command(self):
        msg = Twist()   #Sends different values through the twist matrix
        #By doing ros2 interface show geometry_msgs/msg/Twist we can see what the matrix is made up of
        msg.linear.x = 2.0  #Setting linear velocity value for X
        msg.angular.z = 1.0 #Setting angular velocity value for Z
        self.cmd_vel_pub_.publish(msg)  #Publishes message using .publish function

def main(args=None): #Main code
    rclpy.init(args=args) #Setup
    node = DrawCircleNode() #Runs the node we just made
    rclpy.spin(node)    #Keeps node alive until you kill it
    rclpy.shutdown()    #Shutdown

if __name__ == '__main__':
    main()