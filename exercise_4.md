## Exercise 4: Creating a Service Server

### Goal
Create a service that rotates the turtle to a specific angle.

### Background Information

**Key Concepts:**
- A **service** provides request-response communication
- We'll use the `TeleportAbsolute` service type
- The service takes x, y, and theta parameters
- We'll keep x and y the same, only changing theta (angle)

### Tasks

**Task 4.1:** Create the service script
- Create `rotate_service.py`
- Make it executable

**Task 4.2:** Set up imports and class structure

You'll need:
- Standard ROS imports
- `from turtlesim.srv import TeleportAbsolute`
- `from turtlesim.msg import Pose`
- `import math` (for angle conversions)

**Task 4.3:** Create a service server

In `__init__`, create:
1. A service server named `/rotate_turtle`
2. That uses `TeleportAbsolute` service type
3. With a callback function to handle requests

<details>
<summary> Hint 4.3</summary>

```python
self.srv = self.create_service(
    ServiceType,
    'service_name',
    self.callback_function_name
)
```
</details>

**Task 4.4:** Subscribe to current pose

You need to know the turtle's current position. Create a subscriber to `/turtle1/pose`:
- Store the current pose in `self.current_pose`
- Update it in a callback function

<details>
<summary> Hint 4.4</summary>

```python
self.current_pose = Pose()

self.pose_subscription = self.create_subscription(
    Pose,
    '/turtle1/pose',
    self.pose_callback,
    10
)

def pose_callback(self, msg):
    self.current_pose = msg
```
</details>

**Task 4.5:** Create a client to turtlesim's teleport service

You need to call turtlesim's built-in teleport service:
- Create a client for `/turtle1/teleport_absolute`
- Wait for the service to be available

<details>
<summary> Hint 4.5</summary>

```python
self.teleport_client = self.create_client(
    TeleportAbsolute,
    '/turtle1/teleport_absolute'
)

while not self.teleport_client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Waiting for service...')
```
</details>

**Task 4.6:** Write the service callback function

Create a function that:
1. Takes `request` and `response` as parameters
2. Gets the desired angle from `request.theta`
3. Creates a new request for the teleport service
4. Sets x and y to current position, theta to desired angle
5. Calls the teleport service asynchronously
6. Returns the response

<details>
<summary> Hint 4.6</summary>

```python
def handle_rotate_request(self, request, response):
    desired_angle = request.theta
    
    teleport_request = TeleportAbsolute.Request()
    teleport_request.x = self.current_pose.x
    teleport_request.y = self.current_pose.y
    teleport_request.theta = desired_angle
    
    future = self.teleport_client.call_async(teleport_request)
    
    self.get_logger().info(f'Rotated to {math.degrees(desired_angle):.2f} degrees')
    
    return response
```
</details>

**Task 4.7:** Update setup.py and test

<details>
<summary> Hint 4.7 - Testing</summary>

Terminal 1:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2:
```bash
ros2 run turtle_controller rotate_service
```

Terminal 3 - Call the service:
```bash
# Rotate to 90 degrees (1.57 radians)
ros2 service call /rotate_turtle turtlesim/srv/TeleportAbsolute "{x: 0.0, y: 0.0, theta: 1.57}"
```
</details>
