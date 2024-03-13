from rclpy.node import Node
from std_msgs.msg import Int32
from random import randint


class Ex3PubNode(Node):
    """
    A ROS2 Ex3PubNode node that sends Int32 messages to the '/environment/temperature' and '/environment/humidity' 
    topics, at 1-second intervals.

    The messages contain a fixed message, which demonstrates a simple way to publish data continuously in ROS2.

    Attributes:
        _temp_publisher (Publisher): The ROS2 publisher object for sending messages to '/environment/temperature'.
        _temp_timer (Timer): A timer object that triggers the temp publishing event every 1 second.
        _hum_publisher (Publisher): The ROS2 publisher object for sending messages to '/environment/humidity'.
        _hum_timer (Timer): A timer object that triggers the humidity publishing event every 1 second.
        _temp_msg (Int32): A `std_msgs/msg/Int32` message that holds the temp data to be published.
        _hum_msg (Int32): A `std_msgs/msg/Int32` message that holds the humidity data to be published.

    Args:
        node_name (str): The name of the node.
    """

    def __init__(self, node_name):
        """
        Initializes the publisher node, creates a publisher for the '/environment/temperature' and '/environment/humidity' 
        topics, and starts a timers to publish messages every second.

        Args:
            node_name (str): The name of the node, passed to the parent Node class.
        """
        super().__init__(node_name)
        # Create a publisher object for the '/environment/temperature' topic with a queue size of 10.
        self._temp_publisher = self.create_publisher(Int32, "/environment/temperature", 10)
        # Create a publisher object for the '/environment/humidity' topic with a queue size of 10.
        self._hum_publisher = self.create_publisher(Int32, "/environment/humidity", 10)
        # Create a timer that calls `publish_message` every second.
        self._temp_timer = self.create_timer(1, self.temp_publish_message)
        
        self._hum_timer = self.create_timer(1, self.hum_publish_message)
        
        self._temp_msg = Int32()
        
        self._hum_msg = Int32()

    def temp_publish_message(self):
        """
        Callback function for the timer event. This function constructs the message to be published,
        and logs the message to the ROS2 logger.

        The message is randomly generated temperature values.
        """
        # Initialize the message object that will be published.
        self._temp_msg = Int32()
        # Set the message data.
        self._temp_msg.data = randint(1, 100)
        # Publish the message.
        self._temp_publisher.publish(self._temp_msg)
        # Log the message being published.
        self.get_logger().info(f"Publishing temperature: {self._temp_msg.data}")
        
    def hum_publish_message(self):
        """
        Callback function for the timer event. This function constructs the message to be published,
        and logs the message to the ROS2 logger.

        The message is randomly generated humidity values.
        """
        # Initialize the message object that will be published.
        self._hum_msg = Int32()
        # Set the message data.
        self._hum_msg.data = randint(1, 100)
        # Publish the message.
        self._hum_publisher.publish(self._hum_msg)
        # Log the message being published.
        self.get_logger().info(f"Publishing humidity: {self._hum_msg.data}")
