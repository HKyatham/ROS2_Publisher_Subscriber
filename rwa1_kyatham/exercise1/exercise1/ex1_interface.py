from rclpy.node import Node
from std_msgs.msg import Int32


class Ex1Node(Node):
    """
    A ROS2 Ex1 node that sends string messages to the 'even_number' topic at 1-second intervals and 
    subscribes to the same topic.

    Attributes:
        _publisher (Publisher): The ROS2 publisher object for sending messages.
        _subscriber (Subscriber): The ROS2 subscriber object for receiving messages
        _timer (Timer): A timer object that triggers the publishing event every 1 second.
        _msg (Int): A Int object to temporarily store the received data.
        _int_msg (Int32): A `std_msgs/msg/Int32` message that holds the data to be published.
        _counter (Int): A Int object to generate the data to be published.

    Args:
        node_name (str): The name of the node.
    """

    def __init__(self, node_name):
        """
        Initializes the publisher node, creates a publisher for the 'even_number' topic, and starts a timer to publish messages every second.
        Also creates a subscriber node for 'even_number' topic.

        Args:
            node_name (str): The name of the node, passed to the parent Node class.
        """
        super().__init__(node_name)
        # Create a publisher object for the 'even_number' topic with a queue size of 10.
        self._publisher = self.create_publisher(Int32, "even_number", 10)
        # The queue size is set to 10, which is the size of the message queue.
        self._subscriber = self.create_subscription(
            Int32, "even_number", self.receive_message, 10
        )
        self._msg = 0
        # Create a timer that calls `publish_message` every second.
        self._timer = self.create_timer(1, self.publish_message)
        
        self._int_msg = Int32()
        
        self._counter = 0

    def publish_message(self):
        """
        Callback function for the timer event. This function constructs the message to be published,
        and logs the message to the ROS2 logger.

        The message format is Int32
        a simple message pattern.
        """
        # Initialize the message object that will be published.
        self._int_msg = Int32()
        # Set the message data.
        self._int_msg.data = self._counter
        self._counter+=1
        # Publish the message.
        self._publisher.publish(self._int_msg)
        # Log the message being published.
        self.get_logger().info(f"Publishing: {self._int_msg.data}")
    
    def receive_message(self, msg):
        """Handle incoming messages on the "even_number" topic.

        This function is called when a new message is received on the "even_number" topic. It logs the
        message content using the node's logger when the received number is even.

        Args:
            msg (std_msgs.msg.Int32): The received message object, containing the int32 data.
        """
        #self.get_logger().info(f"Receiving: {msg.data}")
        # Logs the received message data.
        if(msg.data % 2 == 0):
            self._msg = msg.data
            self.get_logger().info(f"Receiving: {self._msg}")