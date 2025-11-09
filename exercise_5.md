## Exercise 5: Creating a Service Client

### Goal
Create a client that calls your rotation service from the command line.

### Background Information

**Key Concepts:**
- A **service client** calls services
- We'll read the desired angle from command line arguments
- Convert degrees to radians for the service call

### Tasks

**Task 5.1:** Create the client script
- Create `rotate_client.py`
- Make it executable

**Task 5.2:** Set up imports

You'll need:
- Standard ROS imports
- `TeleportAbsolute` from turtlesim.srv
- `import math` for angle conversion
- `import sys` for command line arguments

**Task 5.3:** Create a service client

In `__init__`:
- Create a client for `/rotate_turtle` service
- Wait for the service to be available

<details>
<summary> Hint 5.3</summary>

Similar to creating a client in the service server, but simpler since this is the only thing this node does.
</details>

**Task 5.4:** Write a method to send requests

Create a method called `send_request(angle_degrees)` that:
1. Takes an angle in degrees as parameter
2. Creates a service request
3. Converts degrees to radians
4. Calls the service
5. Waits for the response

<details>
<summary> Hint 5.4</summary>

```python
def send_request(self, angle_degrees):
    request = TeleportAbsolute.Request()
    request.x = 0.0  # Not used, but required
    request.y = 0.0  # Not used, but required
    request.theta = math.radians(angle_degrees)
    
    future = self.client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    
    if future.result() is not None:
        self.get_logger().info('Success!')
```
</details>

**Task 5.5:** Handle command line arguments in main()

In the main function:
1. Check if an angle was provided as command line argument
2. If not, print usage instructions
3. If yes, convert to float and call send_request()

<details>
<summary> Hint 5.5</summary>

```python
def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('Usage: ros2 run turtle_controller rotate_client ')
        return
    
    try:
        angle = float(sys.argv[1])
    except ValueError:
        print('Error: Please provide a valid number')
        return
    
    # Create client, send request, cleanup
```
</details>

**Task 5.6:** Update setup.py and test

<details>
<summary> Hint 5.6 - Testing</summary>

Make sure the service server is running, then:
```bash
ros2 run turtle_controller rotate_client 45
ros2 run turtle_controller rotate_client 90
ros2 run turtle_controller rotate_client 180
```
</details>