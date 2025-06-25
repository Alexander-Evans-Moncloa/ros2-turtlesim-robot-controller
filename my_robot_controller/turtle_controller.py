#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  #Import pose from turtlesim messages
from geometry_msgs.msg import Twist #Import twist from geometry_msgs messages
from turtlesim.srv import SetPen    #Here we import the service type for call_set_pen_service(), which is turtlesim/srv/SetPen
from functools import partial #Important function for call_set_pen_service()

class TurtleControllerNode(Node):
    
    #Does a combo of draw_circle and pose_subscriber in terms of subscribing to pose and publishing to cmd_vel
    def __init__(self):
        super().__init__("turtle_controller")   #Name of function
        self.previous_x_ = 0 #Initialisation value for pose_callback() colour changer
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle controller has been started.")   #Print initialisation

    def pose_callback(self, pose: Pose):    #Pose callback function
        cmd = Twist()   #Create variable from the entire Twist class
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0  #Assigns low linear speed
            cmd.angular.z = 0.9 #Assigns turning speed
        else:
            cmd.linear.x = 5.0  #Assigns faster value to linear x
            cmd.angular.z = 0.0 #Assigns null value to angular z
        self.cmd_vel_publisher_.publish(cmd)    #Publishes the new cmd variable back to wherever

        if pose.x > 5.5 and self.previous_x_ <= 5.5: #Prevents service spamming by only changing colour when turtle goes OVER the middle line
            self.previous_x_ = pose.x   #Updates previous_x_ value after the if statement
            self.get_logger().info("Set colour to red!") #Just some logs
            self.call_set_pen_service(255, 0, 0, 3, 0) #If on left side of screen, make line red (width 3)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set colour to green!")
            self.call_set_pen_service(0, 255, 0, 3, 0) #If on right side of screen, make line green (width 3)
        #Services shouldn't be spammed the way topics are. We can find out that this topic is sent 62.5 times a second using ros2 topic hz /turtle1/pose.

    def call_set_pen_service(self, r, g, b, width, off):    #ros2 service type /turtle1/set_pen shows us turtlesim/srv/SetPen. Then ros2 interface show turtlesim/srv/SetPen shows us r, g, b, width and off.
        client = self.create_client(SetPen, "/turtle1/set_pen")   #We provide first service type, then service name. Now we have an object representing the client.
        while not client.wait_for_service(): #Prevents us from calling the service if the service is not up
            self.get_logger().warn("Waiting for service...")    #Prints this in yellow text if the service is not up, every second on the second

        request = SetPen.Request()  #Here we set up a request
        request.r = r   #Here we are filling the request for all the possible parameters
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        #call_async calls the service in an asynchronous way (returns immediately), as opposed to "call" that calls the service then blocks this thread until service replies (can have problems)
        future = client.call_async(request) #Future is an object (of something that will be done in the future)
        future.add_done_callback(partial(self.callback_set_pen))    #Calls the below callback function when the service responds

    def callback_set_pen(self, future): #Callback for when the above service replies with the response
        try:
            response = future.result()  #See if there is a response from future
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))   #Prints to console if there is an exception

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
