#!/usr/bin/env python3
#Remember to use colcon build every time in cmd to update the code, OR enter colcon build --symlink-install and it updates every time you save
import rclpy
from rclpy.node import Node 

#New node I am making, is saved as an object (using OOP)
class MyNode(Node): #Inherits Node from the node class of rclpy. Why? So our class has access to all functionalities of ROS2

    def __init__(self): #Constructor
        super().__init__("first_node") #Calls constructor of the upper class (node class from MyNode(Node))
        #self.get_logger().info("Wazzap from ROS2") #Actually DOES something with the node! (prints the text). Self gets functionalities of node class.

        self.counter_ = 0   #Creates counter and sets it to 0
        self.create_timer(1.0, self.timer_callback) #Creates timer inside a node which says "every 1 second I'm gonna call this function (timer_callback)"

    #Creates callback
    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_)) #Prints "Hello" to terminal
        self.counter_ += 1  #Adds 1 to counter each time this callback is ran


#Main code
def main(args=None):
    rclpy.init(args=args) #Initialises ROS2 communications. Defines args as args from main.

    node = MyNode() #Here I create the node. Nothing in parameters rn, but I can add stuff in the brackets if I want.

    rclpy.spin(node) #Keeps node alive, indefinitely, until I kill it with cntrl+c. All callbacks will be able to be ran.

    rclpy.shutdown() #Closes ROS2 communications

#Calls main code if user enters "__main__"
if __name__ == '__main__':
    main()