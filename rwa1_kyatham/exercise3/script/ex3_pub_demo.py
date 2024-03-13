#!/usr/bin/env python3

"""
This script initializes a ROS2 node using rclpy, which publishes messages using a custom Ex3PubNode.
The Ex3PubNode is defined in the exercise3 package, and it is responsible for publishing messages
to a topic as defined within its implementation. This script is an entry point for running the publisher node,
setting it up, spinning it to keep it alive and processing data, and properly shutting it down afterwards.

Usage:
    To run this script, use the following command in a terminal:
    ```
    ros2 run exercise3 ex3_pub_demo.py
    ```
"""
# Import ROS Client Library for Python
import rclpy

# Import the custom Ex3PubNode class
from exercise3.ex3_pub_interface import (
    Ex3PubNode,
)


def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS client library
    node = Ex3PubNode("ex3_pub")  # Create an instance of the Ex3PubNode
    rclpy.spin(node)  # Keep the node alive to listen for incoming data or events
    node.get_logger().info("Destroying")
    node.destroy_node()  # Properly destroy the node once it's no longer needed
    rclpy.shutdown()  # Shutdown the ROS client library


if __name__ == "__main__":
    main()  # Execute the main function when the script is run
