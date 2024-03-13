#!/usr/bin/env python3

"""
This script initializes a ROS2 node using rclpy, which publishes messages using a custom Ex2Node.
The Ex2Node is defined in the exercise2 package, and it is responsible for publishing messages
to a topic as defined within its implementation. This script is an entry point for running the Ex2Node,
setting it up, spinning it to keep it alive and processing data, and properly shutting it down afterwards.

Usage:
    To run this script, use the following command in a terminal:
    ```
    ros2 run exercise2 ex2_demo.py
    ```
"""
# Import ROS Client Library for Python
import rclpy

# Import the custom Ex2Node class
from exercise2.ex2_interface import (
    Ex2Node,
)


def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS client library
    node = Ex2Node("ex2")  # Create an instance of the Ex2Node
    rclpy.spin(node)  # Keep the node alive to listen for incoming data or events
    node.get_logger().info("Destroying")
    node.destroy_node()  # Properly destroy the node once it's no longer needed
    rclpy.shutdown()  # Shutdown the ROS client library


if __name__ == "__main__":
    main()  # Execute the main function when the script is run
