# Nodes, Topics, Services: The Communication Backbone

The power of ROS 2 lies in its elegant communication architecture that enables complex robotic systems to be built from simple, modular components. At the heart of this architecture are nodes that communicate through topics, services, and actions. Understanding these communication patterns is essential for building robust, maintainable robotic applications.

## Learning Objectives

By the end of this chapter, you will be able to:
1. Design and implement ROS 2 nodes as modular building blocks
2. Create publisher-subscriber systems using topics for asynchronous communication
3. Implement request-response communication patterns using services
4. Design long-running operations with feedback using actions

## Nodes: The Building Blocks of ROS 2

Nodes are the fundamental building blocks of any ROS 2 system. Each node represents an independent process that performs a specific function, such as sensor data processing, control algorithm execution, or user interface management. Nodes encapsulate functionality and communicate with other nodes through the ROS 2 communication infrastructure.

### Node Characteristics

A ROS 2 node typically includes:
- **Node name**: A unique identifier within the ROS 2 graph
- **Parameters**: Configurable values that can be set at runtime
- **Publishers**: Interfaces for sending messages to topics
- **Subscribers**: Interfaces for receiving messages from topics
- **Services**: Interfaces for providing request-response communication
- **Actions**: Interfaces for long-running operations with feedback

> [!NOTE]
> Nodes in ROS 2 are designed to be lightweight and focused on a single responsibility. This design principle, known as the single responsibility principle, makes systems more maintainable and testable.

### Creating Nodes with rclpy

The Python client library for ROS 2 (rclpy) provides the necessary tools to create nodes. Here's a basic structure for a ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize publishers, subscribers, services, etc.
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish-Subscribe Communication (Asynchronous)

Topics form the backbone of ROS 2's asynchronous communication system. In the publish-subscribe pattern, publishers send messages to topics without knowing who will receive them, and subscribers receive messages from topics without knowing who published them. This loose coupling enables flexible, scalable robotic systems.

### Topic Characteristics

Topics in ROS 2 have several important characteristics:
- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic
- **Message types**: Each topic has a specific message type that defines the structure of data being transmitted
- **Quality of Service (QoS)**: Configurable policies for reliability, durability, and performance

> [!TIP]
> Use topics for data that is continuously generated and consumed, such as sensor readings, robot state information, or processed data streams. Topics are ideal for real-time data transmission where the most recent value is typically the most important.

### Publisher Implementation

Here's an example of a publisher node that publishes string messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Implementation

And here's the corresponding subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Response Communication (Synchronous)

Services provide synchronous request-response communication in ROS 2. When a client calls a service, it sends a request and waits for a response from the service server. This pattern is ideal for operations that need to return a result or confirm completion before proceeding.

### Service Characteristics

Services in ROS 2 have these characteristics:
- **Synchronous**: The client waits for the service to complete
- **One-to-one**: Each service call is handled by a single service server
- **Request-Response**: Each call has a defined request and response message type
- **Blocking**: The client thread is blocked until the response is received

> [!WARNING]
> Be careful when using services in time-critical applications, as the blocking nature can cause delays. Consider using actions for long-running operations or topics for continuous data streams.

### Service Implementation

Service definitions are specified in `.srv` files that define the request and response message types. Here's an example service definition:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

Service server implementation:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Long-Running Operations with Feedback

Actions are designed for long-running operations that provide feedback during execution and return a result upon completion. Unlike services, actions don't block the client, allowing for more responsive applications. Actions are ideal for robot navigation, manipulation tasks, or any operation that takes a significant amount of time.

### Action Characteristics

Actions in ROS 2 include:
- **Goal**: The desired outcome of the action
- **Feedback**: Periodic updates on the progress of the action
- **Result**: The final outcome of the action
- **Non-blocking**: The client can continue executing while the action runs

> [!TIP]
> Use actions for operations that take more than a few seconds to complete, such as robot navigation to a goal location, arm manipulation tasks, or complex planning operations. Actions provide a better user experience than services for long-running operations.

### Action Implementation

Action definitions are specified in `.action` files:

```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] feedback
```

## Quality of Service (QoS) Profiles

Quality of Service (QoS) profiles allow you to configure how messages are delivered in ROS 2. Different applications have different requirements for reliability, latency, and data persistence, and QoS profiles provide the flexibility to meet these requirements.

### Common QoS Settings

- **Reliability**: Choose between reliable (guaranteed delivery) or best-effort (faster, no guarantee)
- **Durability**: Choose between transient-local (historical data) or volatile (current data only)
- **History**: Choose between keep-all or keep-last with a specific depth
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is still active

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Example: High-reliability profile for critical data
reliable_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Example: Best-effort profile for real-time sensor data
best_effort_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

> [!NOTE]
> Choosing appropriate QoS settings is crucial for system performance. For sensor data, best-effort reliability may be acceptable to reduce latency. For critical control commands, reliable delivery is essential.

## Custom Message Types

While ROS 2 provides many standard message types, you'll often need to define custom messages for your specific application. Custom messages are defined in `.msg` files and must be built using the ROS 2 build system.

### Creating Custom Messages

1. Create a `msg` directory in your package
2. Define your message in a `.msg` file
3. Update `package.xml` to include message generation dependencies
4. Update `CMakeLists.txt` or `setup.py` to include message generation

Example custom message (`MyRobot.msg`):
```
# Custom robot state message
float64 position_x
float64 position_y
float64 orientation
uint8[] sensor_readings
bool is_moving
```

## Mermaid: Topic Pub/Sub Flow + Service Sequence Diagram

```mermaid
sequenceDiagram
    participant Publisher as Publisher Node
    participant DDS as DDS Middleware
    participant Subscriber as Subscriber Node

    Publisher->>DDS: Publish message to /chatter
    DDS->>Subscriber: Deliver message to /chatter
    Publisher->>DDS: Publish message to /chatter
    DDS->>Subscriber: Deliver message to /chatter

graph LR
    subgraph "Service Request-Response"
        A[Client] -->|Request| B(Service Server)
        B -->|Response| A
    end
```

## Key Takeaways

🧩 **Nodes** are independent processes that encapsulate specific functionality
📡 **Topics** enable asynchronous publish-subscribe communication
🔄 **Services** provide synchronous request-response communication
⏱️ **Actions** support long-running operations with feedback
⚙️ **QoS Profiles** configure communication behavior for different needs
📝 **Custom Messages** allow domain-specific data structures
🎯 **Loose Coupling** enables flexible, scalable robotic systems

## Further Reading

1. [ROS 2 Topics and Services](https://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html) - Official tutorial on topics
2. [ROS 2 Actions Guide](https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html) - Comprehensive guide to actions
3. [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html) - Detailed QoS documentation

## Assessment

### Multiple Choice Questions

1. What is the main difference between topics and services in ROS 2?
   A) Topics are faster than services
   B) Topics are asynchronous, services are synchronous
   C) Topics use more memory than services
   D) Topics are only for sensor data

2. Which communication pattern is best for long-running operations that provide feedback?
   A) Topics
   B) Services
   C) Actions
   D) Parameters

3. What does QoS stand for in ROS 2?
   A) Quality of Service
   B) Quick Operating System
   C) Query Optimization System
   D) Quantitative Operating Service

4. In the publish-subscribe pattern, what is true about publishers and subscribers?
   A) Publishers must know about all subscribers
   B) Subscribers must know about all publishers
   C) Publishers and subscribers are loosely coupled
   D) Publishers and subscribers must run simultaneously

### Exercises

1. Create a custom message type for a robot's sensor data that includes readings from an IMU, a LiDAR, and a camera. Implement a publisher that sends this custom message and a subscriber that processes it.

2. Implement a service that takes two numbers as input and returns their product. Test the service using both a custom client and the command-line tool `ros2 service call`.

### Mini-Project

Develop a complete ROS 2 system with at least 3 nodes that communicate using different patterns:
1. A sensor node that publishes data using topics
2. A processing node that subscribes to sensor data and provides a service for data analysis
3. A control node that uses actions to execute long-running tasks based on processed data

> [!SOLUTION]
> Solution: The project should include:
> 1. Three properly structured nodes with clear responsibilities
> 2. Topic-based communication for sensor data
> 3. Service-based communication for data analysis
> 4. Action-based communication for long-running tasks
> 5. Proper error handling and logging
> 6. Documentation explaining the communication patterns used