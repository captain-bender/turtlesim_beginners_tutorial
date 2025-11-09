## Exercise 2: Creating a Topic Publisher

### Goal
Create a node that publishes velocity commands to make the turtle move in a circle.

### Background Information

**Key Concepts:**
- A **publisher** sends messages to a topic
- The turtlesim listens to `/turtle1/cmd_vel` for velocity commands
- Velocity commands use the `Twist` message type
- `Twist` has two main parts:
  - `linear` - forward/backward movement (we use `linear.x`)
  - `angular` - rotational movement (we use `angular.z`)

### Tasks

**Task 2.1:** Create the Python script file
- Navigate to `~/turtle_ws/src/turtle_controller/turtle_controller/`
- Create a file named `velocity_publisher.py`
- Make it executable

<details>
<summary> Hint 2.1</summary>

Use `nano` or your favorite text editor to create the file.
Use `chmod +x filename.py` to make it executable.
</details>

**Task 2.2:** Write the basic node structure

Write a Python script that includes:
1. Shebang line for Python3
2. Import necessary modules (rclpy, Node, Twist)
3. A class that inherits from `Node`
4. A main function that initializes ROS, creates the node, and spins

<details>
<summary> Hint 2.2 - Imports</summary>

You need to import:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
```
</details>

<details>
<summary> Hint 2.2 - Class Structure</summary>

```python
class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('node_name_here')
        # Your code here
```
</details>

<details>
<summary> Hint 2.2 - Main Function</summary>

```python
def main(args=None):
    rclpy.init(args=args)
    node = YourClassName()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>

**Task 2.3:** Create a publisher in the `__init__` method

Add code to create a publisher that:
- Publishes to the topic `/turtle1/cmd_vel`
- Uses the `Twist` message type
- Has a queue size of 10

<details>
<summary> Hint 2.3</summary>

Use the `create_publisher()` method:
```python
self.publisher_ = self.create_publisher(MessageType, 'topic_name', queue_size)
```
</details>

**Task 2.4:** Create a timer

Add a timer that calls a callback function every 0.5 seconds.

<details>
<summary> Hint 2.4</summary>

Use the `create_timer()` method:
```python
timer_period = 0.5  # seconds
self.timer = self.create_timer(timer_period, self.callback_function_name)
```
</details>

**Task 2.5:** Write the timer callback function

Create a method that:
1. Creates a new `Twist` message
2. Sets `linear.x` to 2.0 (move forward)
3. Sets `angular.z` to 1.0 (turn counter-clockwise)
4. Publishes the message
5. Logs the action using `self.get_logger().info()`

<details>
<summary> Hint 2.5 - Creating Twist Message</summary>

```python
def timer_callback(self):
    msg = Twist()
    msg.linear.x = # Your value here
    msg.angular.z = # Your value here
    # Publish and log
```
</details>

<details>
<summary> Hint 2.5 - Publishing</summary>

```python
self.publisher_.publish(msg)
```
</details>

**Task 2.6:** Register the node in setup.py

Edit the `setup.py` file to add an entry point for your node.

<details>
<summary> Hint 2.6</summary>

Find the `entry_points` section and add:
```python
entry_points={
    'console_scripts': [
        'velocity_publisher = turtle_controller.velocity_publisher:main',
    ],
},
```
</details>

**Task 2.7:** Build and test

- Build the package
- Source the workspace
- Test your publisher

<details>
<summary> Hint 2.7 - Testing</summary>

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:
```bash
ros2 run turtle_controller velocity_publisher
```

The turtle should move in a circle!
</details>