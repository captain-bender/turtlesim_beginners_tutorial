## Exercise 3: Creating a Topic Subscriber

### Goal
Create a node that subscribes to the turtle's pose and prints its position.

### Background Information

**Key Concepts:**
- A **subscriber** receives messages from a topic
- The turtlesim publishes pose information to `/turtle1/pose`
- The `Pose` message contains: x, y, theta, linear_velocity, angular_velocity

### Tasks

**Task 3.1:** Create the subscriber script
- Create `pose_subscriber.py` in the `turtle_controller` directory
- Make it executable

**Task 3.2:** Write the basic structure

Similar to the publisher, create:
1. Shebang and imports (need `turtlesim.msg import Pose` instead of Twist)
2. A class inheriting from Node
3. Main function

<details>
<summary> Hint 3.2 - Imports</summary>

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
```
</details>

**Task 3.3:** Create a subscriber in `__init__`

Create a subscriber that:
- Subscribes to `/turtle1/pose`
- Uses the `Pose` message type
- Calls a callback function when messages arrive
- Has a queue size of 10

<details>
<summary> Hint 3.3</summary>

Use the `create_subscription()` method:
```python
self.subscription = self.create_subscription(
    MessageType,
    'topic_name',
    self.callback_function_name,
    queue_size
)
```
</details>

**Task 3.4:** Write the callback function

Create a method that:
- Takes `msg` as a parameter (the received Pose message)
- Logs the turtle's x, y, and theta values
- Use formatted strings to display values nicely

<details>
<summary> Hint 3.4</summary>

```python
def pose_callback(self, msg):
    self.get_logger().info(f'X: {msg.x:.2f}, Y: {msg.y:.2f}, Theta: {msg.theta:.2f}')
```
</details>

**Task 3.5:** Update setup.py and test

- Add the entry point for `pose_subscriber`
- Build and source
- Test with turtlesim and turtle_teleop_key

<details>
<summary> Hint 3.5 - Testing</summary>

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:
```bash
ros2 run turtle_controller pose_subscriber
```

Terminal 3:
```bash
ros2 run turtlesim turtle_teleop_key
```

Use arrow keys to move the turtle and watch the pose output!
</details>
