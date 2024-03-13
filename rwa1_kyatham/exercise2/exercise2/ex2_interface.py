from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float32


class Ex2Node(Node):
    """
    A ROS2 Ex2 node that sends string messages to the 'even_number' topic at 1-second intervals, subscribes to the same topic
    and 'divided_number' topic whenever 'even_number' topic publishes a even number.

    Attributes:
        _publisher (Publisher): The ROS2 publisher object for sending messages.
        _divide_publisher (Publisher): The ROS2 publisher object for sending messages.
        _subscriber (Subscriber): The ROS2 subscriber object for receiving messages.
        _timer (Timer): A timer object that triggers the publishing event every 1 second.
        _msg (Int): A Int object which temporarily stores the received data.
        _int_msg (Int32): A `std_msgs/msg/Int32` message that holds the data to be published.
        _float_msg (Float32): A `std_msgs/msg/Float32` message that holds the data to be published.
        _counter (Int): Int object which generates data to publish.
        

    Args:
        node_name (str): The name of the node.
    """

    def __init__(self, node_name):
        """
        Initializes the Ex2 node, creates publisher for the 'even_number' and 'divided_number' topic, 
        and starts a timer to publish messages every second. Also creates a subscriber for 'even_number'

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
        self._divide_publisher = self.create_publisher(Float32, "divided_number", 10)
        self._msg = 0
        # Create a timer that calls `publish_message` every second.
        self._timer = self.create_timer(1, self.publish_message)
        
        self._int_msg = Int32()
        
        self._float_msg = Float32()
        
        self._counter = 0

    def publish_message(self):
        """
        Callback function for the timer event. This function constructs the message to be published,
        and logs the message to the ROS2 logger.

        The message format is Int32.
        """
        # Initialize the message object that will be published.
        self._int_msg = Int32()
        # Set the message data.
        self._int_msg.data = self._counter
        self._counter+=1
        # Publish the message.
        self._publisher.publish(self._int_msg)
        # Log the message being published.
        self.get_logger().info(f"Publishing to even_number: {self._int_msg.data}")
        
    def publish_divide_message(self, float_data):
        """
        Callback function for the timer event. This function constructs the message to be published,
        and logs the message to the ROS2 logger.

        The message format is Float32.
        Args:
            float_data (float): The data that needs to be published to 'divide_number'.
        """
        # Initialize the message object that will be published.
        self._float_msg = Float32()
        # Set the message data.
        self._float_msg.data = float_data
        # Publish the message.
        self._divide_publisher.publish(self._float_msg)
        # Log the message being published.
        self.get_logger().info(f"Publishing to divided_number: {self._float_msg.data}")
    
    def receive_message(self, msg):
        """Handle incoming messages on the "even_number" topic.

        This function is called when a new message is received on the "even_number" topic. It checks 
        if the received number is even, if it's even it publishes to divide_number topic.

        Args:
            msg (std_msgs.msg.Int32): The received message object, containing the int32 data.
        """
        #self.get_logger().info(f"Receiving: {msg.data}")
        # Logs the received message data.
        if(msg.data % 2 == 0):
            self._msg = msg.data
            self.get_logger().info(f"Receiving and processing from even_number: {self._msg}")
            float_data = self._msg/2.0
            self.publish_divide_message(float_data)