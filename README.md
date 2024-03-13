# ROS2 Publisher and Subscriber Examples.

This Project is a ROS2 Galactic Package with publisher and subscriber examples where in multiples nodes are created which either publish or subscribe to a topic or do both at the same time. There is alsoa node which processes the published information and processes it and logs it at specific intervals of time.

This project has 3 different packages.

## First Package - Exercise 1
In this package the goal is 
The publisher publishes messages of type std_msgs/msg/Int32 to the topic even_number . The node should publish a sequence of integers (one message is an integer), starting from 0 and incrementing by 1 at a frequency of 1 Hz. In the same node, create a subscriber to subscribe to topic even_number . The subscriber should only process and print messages to the console if the integer is even. Add detailed logging. The publisher should log each number it publishes, and the subscriber should log when it receives and processes an even number.

To run this package, 

clone the project using the command, to your src folder of your ROS workspace.
```
git clone 
```

Build the project using the colcon build.
```
colcon build --packages-select exercise1
```

Source the package using the command
```
source install/setup.bash
```

Run the node using the ros2 run command
```
ros2 run exercise1 ex1_demo.py
```

The output looks something like below:
![Images/Exercise1.png](https://github.com/HKyatham/ROS2_Publisher_Subscriber/blob/main/Images/Exercise1.png)

## First Package - Exercise 2
In this package the goal is to create two publishers and a single subscriber. The publisher which publishes to topic "divided_number" can be called directly from the subscriber callback function or from another timer. Task is also to determine the appropriate message type for the topic divided_number topic on your own. This message type should be part of std_msgs. Add detailed logging. Publishers must log every number they publish, while thesubscriber should record each message it receives.

To run this package, 

clone the project using the command, to your src folder of your ROS workspace.
```
git clone 
```

Build the project using the colcon build.
```
colcon build --packages-select exercise2
```

Source the package using the command
```
source install/setup.bash
```

Run the node using the ros2 run command
```
ros2 run exercise1 ex2_demo.py
```

The output looks something like below:
![Images/Exercise2.png](https://github.com/HKyatham/ROS2_Publisher_Subscriber/blob/main/Images/Exercise2.png)

## First Package - Exercise 3
In this package the goal is to create two nodes, one for the publisher and one for the subscribers. Every second, the publisher publishes random temperature (°C) values on topic /environment/temperature and humidity (%) values on topic /environment/humidity. Random values for both temperature and humidity are produced within the range [0,100]. Subscribe to the topic /environment/temperature and topic /environment/humidity.
Process the incoming data to evaluate comfort levels based on simple conditions:
Temperature (°C) Comfort Levels:
Cold: < 15°C, Comfortable: 15°C– 25°C, Hot: > 25°C
Humidity (%) Comfort Levels:
Dry: < 30%, Comfortable: 30% – 60%, Humid: > 60%
Print a message in the terminal indicating the current comfort level, e.g., “Comfort Level: Comfortable (temperature) and Dry (humidity).”
In the subscriber node, use a timer to process data retrieved from the subscriber callbacks and prints the the comfort level.

To run this package, 

clone the project using the command, to your src folder of your ROS workspace.
```
git clone 
```

Build the project using the colcon build.
```
colcon build --packages-select exercise3
```

Source the package using the command
```
source install/setup.bash
```

Run the node using the ros2 run command in one terminal
```
ros2 run exercise3 ex3_pub_demo.py
```

Open another terminal and run the ros2 run command
```
ros2 run exercise3 ex3_sub_demo.py

```

The output looks something like below:
![Images/Exercise3.png](https://github.com/HKyatham/ROS2_Publisher_Subscriber/blob/main/Images/Exercise3.png)
