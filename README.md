
# ROS2 Turtlesim Robot Controller

A fully commented robot controller for the Turtlesim simulation environment in ROS2. This project is based on the original code by RoboticsBackend.

![Turtlesim Robot Controller Screenshot](https://github.com/Alexander-Evans-Moncloa/ros2-turtlesim-robot-controller/blob/main/Screenshot%20from%202025-06-25%2012-58-52.png)

## Features

- Control a turtle in the Turtlesim environment.
- Fully commented code for easy understanding and modification.
- Simple and intuitive interface for robot control.

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

## Contributing

Contributions are welcome! If you have suggestions for improvements or new features, please open an issue or submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- Special thanks to RoboticsBackend for the original code and inspiration.
