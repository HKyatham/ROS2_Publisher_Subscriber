from rclpy.node import Node
from std_msgs.msg import Int32


class Ex3SubNode(Node):
    """A ROS2 Ex3SubNode node that listens to string messages on a specified topic.

    This class creates a node that subscribes to the "/environment/temperature" and "/environment/humidity" 
    topics, expecting messages of type `std_msgs/msg/Int32`.
    It logs the received messages to the ROS2 logger.

    Attributes:
        _temp_subscriber (rclpy.subscription.Subscription): The subscription object for receiving Int32 messages.
        _hum_subscriber (rclpy.subscription.Subscription): The subscription object for receiving Int32 messages.
        _temp_msg (Int): A Int to store temp values from /environment/temperature topic.
        _hum_msg (Int):A Int to store humidity values from /environment/humidity topic.
        _temp_comfort (String): A String to store the processed temp comfort levels.
        _hum_comfort (String): A String to store the processed humidity comfort levels.
        _timer (Timer): A timer object that triggers the publishing event every 1 second.

    Args:
        node_name (str): The name of the node, provided during instantiation.
    """

    def __init__(self, node_name):
        """Initialize the Ex3SubNode with a given name and create a subscription to the "/environment/temperature" 
        and "/environment/humidity" topics.

        The method sets up a subscriber for the "/environment/temperature" and "/environment/humidity" topics 
        with a string message type. It specifies callback functions that is invoked upon receiving a message.

        Args:
            node_name (str): The name of the node.
        """
        super().__init__(node_name)
        self._temp_subscriber = self.create_subscription(
            Int32, "/environment/temperature", self.receive_temp_message, 10
        )
        self._hum_subscriber = self.create_subscription(
            Int32, "/environment/humidity", self.receive_hum_message, 10
        )
        self._temp_msg = 0
        self._hum_msg = 0
        # The queue size is set to 10, which is the size of the message queue.
        self._temp_comfort = ""
        self._hum_comfort = ""
        self._timer = self.create_timer(1, self.print_message)

    def receive_temp_message(self, msg):
        """Handle incoming messages on the "/environment/temperature" topic.

        This function is called when a new message is received on the "/environment/temperature" topic. It stores this data 
        and processes it to define the comfort levels.

        Args:
            msg (std_msgs.msg.Int32): The received message object, containing the Int32 data.
        """
        #self.get_logger().info(f"Receiving: {msg.data}")
        self._temp_msg = msg.data
        # Logs the received message data.
        if(self._temp_msg < 15):
            #self.get_logger().info("Cold")
            self._temp_comfort = "Cold"
            # End of if case
        elif(self._temp_msg > 15 and self._temp_msg < 25):
            #self.get_logger().info("Comfortable")
            self._temp_comfort = "Comfortable"
            # End of else if case
        elif(self._temp_msg > 25):
            self._temp_comfort = "Hot"
            # End of else if case
    
    def receive_hum_message(self, msg):
        """Handle incoming messages on the "/environment/humidity" topic.

        This function is called when a new message is received on the "/environment/humidity" topic. It stores this data 
        and processes it to define the comfort levels.

        Args:
            msg (std_msgs.msg.Int32): The received message object, containing the Int32 data.
        """
        #self.get_logger().info(f"Receiving: {msg.data}")
        self._hum_msg = msg.data
        # Logs the received message data.
        if(self._hum_msg < 30):
            #self.get_logger().info("Dry")
            self._hum_comfort = "Dry"
            # End of if case
        elif(self._hum_msg > 30 and self._hum_msg < 60):
            #self.get_logger().info("Comfortable")
            self._hum_comfort = "Comfortable"
            # End of else if case
        elif(self._hum_msg > 60):
            self._hum_comfort = "Humid"
            # End of else if case
    
    def print_message(self):
        """Print the comfort levels to the terminal.

        Callback function for the timer event. This function constructs the message to 
        log the message to the ROS2 logger.

        """
        self.get_logger().info(f"Comfort Level: {self._temp_comfort} {self._temp_msg}C and {self._hum_comfort} {self._hum_msg}%.")
