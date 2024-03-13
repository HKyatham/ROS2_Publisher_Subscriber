#!/usr/bin/env python3

"""
This script initializes a ROS2 node using rclpy, which publishes messages using a custom Ex1Node.
The Ex1Node is defined in the exercise1 package, and it is responsible for publishing messages
to a topic as defined within its implementation. This script is an entry point for running the publisher node,
setting it up, spinning it to keep it alive and processing data, and properly shutting it down afterwards.

Usage:
    To run this script, use the following command in a terminal:
    ```
    ros2 run exercise1 ex1_demo.py
    ```
"""
# Import ROS Client Library for Python
import rclpy

# Import the custom Ex1Node class
from exercise1.ex1_interface import (
    Ex1Node,
)


def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS client library
    node = Ex1Node("ex1")  # Create an instance of the Ex1Node
    rclpy.spin(node)  # Keep the node alive to listen for incoming data or events
    node.get_logger().info("Destroying")
    node.destroy_node()  # Properly destroy the node once it's no longer needed
    rclpy.shutdown()  # Shutdown the ROS client library


if __name__ == "__main__":
    main()  # Execute the main function when the script is run
