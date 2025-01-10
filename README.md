# ROS & Robotics Assessment Solutions

## Overview

This repository contains our solutions to the ROS & Robotics Assessment provided by ARTPARK. The assessment covers various topics related to ROS (Robot Operating System) and robotics, including node communication, robot state publishing, teleoperation, collision avoidance, and dynamic node handling. The solutions are implemented in Python and include code snippets, terminal outputs, and screenshots as required.

## Contributors

- [Yousef H. Abdelhady]
- [Abdulrahman Helal]

## Task Explaination


### **Task 1: Simple Publisher/Subscriber**
- **Objective**: 
  - Create a basic ROS communication system between two nodes using the publisher-subscriber model.
- **Tasks**:
  1. Write a ROS node (`publisher_node.py`) that publishes a string message `"Robot is running"` to the topic `/robot_status` at a rate of 10 Hz.
  2. Write another ROS node (`subscriber_node.py`) that subscribes to the `/robot_status` topic and prints the received messages to the console.
  3. Include the complete Python code for both nodes in the document.
  4. Provide a screenshot of the terminal window showing both nodes running with the expected output.

---

### **Task 2: Robot State Publishing**
- **Objective**:
  - Simulate the position of a robot in a 2D environment and publish its pose to a ROS topic.
- **Tasks**:
  1. Write a ROS node (`robot_state_publisher.py`) that publishes the robot's current simulated position (x, y, theta) to the topic `/robot_pose`.
  2. Use the `geometry_msgs/PoseStamped` message type to represent the robot's pose.
  3. Include the complete Python code for the node in the document.
  4. (Optional) If you have access to Gazebo or RViz, run the node and visualize the robot's position. Include a screenshot of the visualization.

---

### **Task 3: TurtleBot3 Teleoperation**
- **Objective**:
  - Teleoperate a simulated TurtleBot3 robot in Gazebo using keyboard commands.
- **Tasks**:
  1. Launch the TurtleBot3 simulation in Gazebo using the `turtlebot3_gazebo` launch file.
  2. Launch the `teleop_twist_keyboard` node to control the robot.
  3. Teleoperate the TurtleBot3 using the keyboard commands.
  4. Include a screenshot of the Gazebo simulation showing the TurtleBot3.
  5. Include a screenshot of the `teleop_twist_keyboard` terminal window.
  6. Briefly describe the keyboard commands used and the observed robot behavior.

---

### **Task 4: Gazebo Robot Collision Avoidance**
- **Objective**:
  - Enable two robots in a Gazebo simulation to avoid colliding with each other.
- **Tasks**:
  1. Describe a simple approach to implement collision avoidance between the two robots.
  2. Consider using robot position information (e.g., from `/gazebo/model_states`) to calculate distances and adjust robot velocities.
  3. Explain your approach in detail.
  4. (Optional) Provide a conceptual outline of how you would implement this in a ROS node (e.g., subscribing to robot positions, calculating distances, and adjusting velocities).

---

### **Task 5: Dynamic Node Handling**
- **Objective**:
  - Dynamically discover and interact with new nodes that appear on the ROS network.
- **Tasks**:
  1. Describe a mechanism to dynamically discover new nodes and automatically subscribe to their topics.
  2. Consider using ROS tools or APIs (e.g., ROS Master API, parameter server) to implement this mechanism.
  3. Explain your approach in detail.
  4. Briefly discuss any relevant ROS tools or APIs that could be used.


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Feel free to reach out to us if you have any questions or need further clarification on any part of the solution.

Happy coding!