# Chapter 4: Creating and Running Simple ROS 2 Nodes in Simulation

## Overview
This chapter guides you through creating and running ROS 2 nodes in a simulated environment. You'll learn to build applications that can communicate and control robots in simulation.

## Key Concepts
- **ROS 2 Packages**: Organized collections of ROS 2 code
- **Message Types**: Standardized data formats for ROS 2 communication
- **Node Composition**: Building applications from multiple ROS 2 nodes

## Learning Outcomes
By the end of this chapter, you will:
1. Create a ROS 2 package
2. Write publisher and subscriber nodes
3. Run nodes in a simulated environment

## Content Coming Soon
Step-by-step guides and complete working examples.

## Code Example
```python
# ROS 2 subscriber node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Next Steps
- Create your first ROS 2 package
- Run the examples in simulation
- Move to the Simulation module
