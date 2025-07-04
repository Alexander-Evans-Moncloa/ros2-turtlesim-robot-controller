
# ROS2 Turtlesim Robot Controller

A fully commented robot controller for the Turtlesim simulation environment in ROS2.

![Turtlesim Robot Controller Screenshot](https://github.com/Alexander-Evans-Moncloa/ros2-turtlesim-robot-controller/blob/main/Screenshot%20from%202025-06-25%2012-58-52.png)

## Features

- **Node**: `turtle_controller`
  - This node is responsible for controlling the turtle's movement and drawing in the Turtlesim environment.

- **Topics**:
  - **`/turtle1/cmd_vel`**: This topic is used to publish velocity commands to the turtle. The `Twist` message type is utilised to control linear and angular speeds.
  - **`/turtle1/pose`**: This topic subscribes to the turtle's pose information, allowing the controller to make decisions based on the turtle's current position.

- **Services**:
  - **`/turtle1/set_pen`**: This service is called to change the pen colour and width when the turtle crosses a specified line on the screen. The service uses the `SetPen` message type, allowing for dynamic drawing capabilities.

## Prerequisites

Before running the project, ensure you have the following installed:

- ROS2 (Humble Hawksbill)
- Turtlesim package
- Linux Ubuntu 22.04 (Jammy Jellyfish)

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Alexander-Evans-Moncloa/ros2-turtlesim-robot-controller.git
   cd ros2-turtlesim-robot-controller
   ```

2. Build the package:
   ```bash
   colcon build --symlink-install
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

To run the Turtlesim simulation and the robot controller, follow these steps:

1. Start the Turtlesim node:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. Run the robot controller:
   ```bash
   ros2 run my_robot_controller turtle_controller
   ```

## License

This project is licensed under the [MIT License].

## Acknowledgments

- Special thanks to RoboticsBackend for the original code and inspiration.
