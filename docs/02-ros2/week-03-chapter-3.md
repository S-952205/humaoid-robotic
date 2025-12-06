# Chapter 3: ROS 2 Basics and Node Communication

## Overview
ROS 2 (Robot Operating System 2) is the middleware that allows different components of a robot to communicate. This chapter introduces ROS 2 concepts and how nodes communicate with each other.

## Key Concepts
- **ROS 2 Node**: A basic computational unit in ROS 2
- **Topics**: Named buses for publishing and subscribing to messages
- **Services**: Request-response communication pattern
- **Actions**: Long-running goal-oriented tasks

## Learning Outcomes
By the end of this chapter, you will understand:
1. What ROS 2 is and how it works
2. How ROS 2 nodes communicate via topics and services
3. How to write and run simple ROS 2 programs

## Content Coming Soon
Detailed guides and working code examples.

## Code Example
```python
# Basic ROS 2 node
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def publish_message(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
```

## Next Steps
- Set up ROS 2 on your system
- Run the provided examples
- Move to Chapter 4: Creating ROS 2 Nodes in Simulation
