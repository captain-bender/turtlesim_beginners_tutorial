# ROS 2 Humble Beginner Tutorial with Turtlesim

## Prerequisites
- ROS 2 Humble installed and sourced
- Basic knowledge of Python
- Use VSCode instead of nano editor in order to work directly inside the docker.
- Start the Xlaunch in order to view the turtlesim graphics.
- A terminal/command line interface

## Table of Contents
1. [Creating a Workspace](#1-creating-a-workspace)
2. [Creating a Package](#2-creating-a-package)
3. [Building and Installing the Package](#3-building-and-installing-the-package)
4. [Creating a Topic Publisher](#4-creating-a-topic-publisher)
5. [Creating a Topic Subscriber](#5-creating-a-topic-subscriber)
6. [Creating a Service Server](#6-creating-a-service-server)
7. [Creating a Service Client](#7-creating-a-service-client)
8. [Testing Everything Together](#8-testing-everything-together)
9. [Creating a Launch File](#9-creating-a-launch-file)

---

## 1. Creating a Workspace

A **workspace** is a directory where you organize and build your ROS 2 packages.

### Step 1.1: Create the workspace directory structure

```bash
# Create the workspace root directory
mkdir -p ~/turtle_ws/src

# Navigate to the workspace
cd ~/turtle_ws
```

**Explanation:**
- `~/turtle_ws` is your workspace root directory
- `src` is where all your ROS 2 packages will live
- The `-p` flag creates parent directories if they don't exist

---

## 2. Creating a Package

A **package** is the basic unit of organization in ROS 2. It contains your code, configuration files, and dependencies.

### Step 2.1: Navigate to the src directory

```bash
cd ~/turtle_ws/src
```

### Step 2.2: Create a new Python package

```bash
ros2 pkg create --build-type ament_python turtle_controller \
  --dependencies rclpy geometry_msgs turtlesim
```

**Explanation:**
- `ros2 pkg create`: Command to create a new package
- `--build-type ament_python`: Specifies we're creating a Python package
- `turtle_controller`: The name of our package
- `--dependencies`: Lists the packages our package depends on
  - `rclpy`: ROS 2 Python client library
  - `geometry_msgs`: Contains message types like Twist (for velocity commands)
  - `turtlesim`: The turtle simulator package

### Step 2.3: Understand the package structure

After creation, your package structure looks like this:

```
turtle_controller/
├── package.xml          # Package metadata and dependencies
├── setup.py            # Python package setup file
├── setup.cfg           # Configuration for Python package
├── resource/           # Resource files
├── test/              # Test files
└── turtle_controller/ # Python module directory (where your code goes)
    └── __init__.py
```

---

## 3. Building and Installing the Package

### Step 3.1: Navigate to workspace root

```bash
cd ~/turtle_ws
```

### Step 3.2: Build the package

```bash
colcon build
```

**Explanation:**
- `colcon`: The build tool for ROS 2
- This command compiles your code and sets up installation files

You should see output like:
```
Starting >>> turtle_controller
Finished <<< turtle_controller [0.50s]

Summary: 1 package finished [0.70s]
```

### Step 3.3: Source the workspace

```bash
source install/setup.bash
```

**Important:** You need to source this file in **every new terminal** where you want to use your package, or add it to your `~/.bashrc`:

```bash
echo "source ~/turtle_ws/install/setup.bash" >> ~/.bashrc
```

---

## 4. Creating a Topic Publisher

A **publisher** sends messages to a topic. We'll create a node that publishes velocity commands to move the turtle.

### Step 4.1: Create the publisher script

```bash
cd ~/turtle_ws/src/turtle_controller/turtle_controller
nano velocity_publisher.py
```

### Step 4.2: Write the publisher code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityPublisher(Node):
    """
    A ROS 2 node that publishes velocity commands to make the turtle move in a circle.
    """
    
    def __init__(self):
        # Initialize the node with a name
        super().__init__('velocity_publisher')
        
        # Create a publisher
        # - Topic: '/turtle1/cmd_vel' (turtlesim's velocity command topic)
        # - Message type: Twist (contains linear and angular velocity)
        # - Queue size: 10 (number of messages to buffer)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create a timer that calls timer_callback every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Velocity Publisher has started!')
    
    def timer_callback(self):
        """
        This function is called every 0.5 seconds by the timer.
        It creates and publishes a Twist message to move the turtle.
        """
        # Create a new Twist message
        msg = Twist()
        
        # Set linear velocity (forward/backward movement)
        msg.linear.x = 2.0  # Move forward at 2.0 units/sec
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Set angular velocity (rotation)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0  # Rotate counter-clockwise at 1.0 rad/sec
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the action
        self.get_logger().info(f'Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our node
    velocity_publisher = VelocityPublisher()
    
    # Keep the node running and processing callbacks
    # This will run until you press Ctrl+C
    rclpy.spin(velocity_publisher)
    
    # Cleanup
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4.3: Make the script executable

```bash
chmod +x velocity_publisher.py
```

### Step 4.4: Update setup.py

Edit the setup.py file to register our new node:

```bash
cd ~/turtle_ws/src/turtle_controller
nano setup.py
```

Find the `entry_points` section and modify it:

```python
entry_points={
    'console_scripts': [
        'velocity_publisher = turtle_controller.velocity_publisher:main',
    ],
},
```

**Explanation:**
- This registers `velocity_publisher` as a command you can run with `ros2 run`
- Format: `'command_name = package_name.script_name:function_name'`

### Step 4.5: Build and test

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

**Test the publisher:**

Terminal 1 - Start turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2 - Run your publisher:
```bash
ros2 run turtle_controller velocity_publisher
```

You should see the turtle moving in a circle!

---

## 5. Creating a Topic Subscriber

A **subscriber** receives messages from a topic. We'll create a node that listens to the turtle's pose (position and orientation).

### Step 5.1: Create the subscriber script

```bash
cd ~/turtle_ws/src/turtle_controller/turtle_controller
nano pose_subscriber.py
```

### Step 5.2: Write the subscriber code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseSubscriber(Node):
    """
    A ROS 2 node that subscribes to the turtle's pose and prints it.
    """
    
    def __init__(self):
        # Initialize the node
        super().__init__('pose_subscriber')
        
        # Create a subscriber
        # - Topic: '/turtle1/pose' (turtlesim publishes pose here)
        # - Message type: Pose (contains x, y, theta, linear_velocity, angular_velocity)
        # - Callback function: pose_callback (called when a message is received)
        # - Queue size: 10
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Pose Subscriber has started!')
    
    def pose_callback(self, msg):
        """
        This function is called automatically whenever a message is received on /turtle1/pose.
        
        Args:
            msg (Pose): The received pose message
        """
        # Log the turtle's position and orientation
        self.get_logger().info(
            f'Turtle Pose - X: {msg.x:.2f}, Y: {msg.y:.2f}, Theta: {msg.theta:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    
    # Keep the node running and processing incoming messages
    rclpy.spin(pose_subscriber)
    
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5.3: Make executable and update setup.py

```bash
chmod +x pose_subscriber.py
cd ~/turtle_ws/src/turtle_controller
nano setup.py
```

Update entry_points:

```python
entry_points={
    'console_scripts': [
        'velocity_publisher = turtle_controller.velocity_publisher:main',
        'pose_subscriber = turtle_controller.pose_subscriber:main',
    ],
},
```

### Step 5.4: Build and test

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

**Test the subscriber:**

Terminal 1 - Turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2 - Your subscriber:
```bash
ros2 run turtle_controller pose_subscriber
```

Terminal 3 - Control the turtle manually:
```bash
ros2 run turtlesim turtle_teleop_key
```

Use arrow keys to move the turtle and watch the subscriber print the pose!

---

## 6. Creating a Service Server

A **service** provides request-response communication. We'll create a service server that rotates the turtle by a specified angle.

### Step 6.1: Create the service server script

```bash
cd ~/turtle_ws/src/turtle_controller/turtle_controller
nano rotate_service.py
```

### Step 6.2: Write the service server code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
import math


class RotateService(Node):
    """
    A ROS 2 service server that rotates the turtle to a specific angle.
    """
    
    def __init__(self):
        super().__init__('rotate_service')
        
        # Create a service server
        # - Service name: '/rotate_turtle'
        # - Service type: TeleportAbsolute (we'll use it to set the angle)
        # - Callback: handle_rotate_request
        self.srv = self.create_service(
            TeleportAbsolute,
            '/rotate_turtle',
            self.handle_rotate_request
        )
        
        # Subscribe to the turtle's current pose so we know its position
        self.current_pose = Pose()
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # Create a client to call turtlesim's teleport service
        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute'
        )
        
        # Wait for the teleport service to be available
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /turtle1/teleport_absolute service...')
        
        self.get_logger().info('Rotate Service is ready!')
    
    def pose_callback(self, msg):
        """Update the current pose of the turtle."""
        self.current_pose = msg
    
    def handle_rotate_request(self, request, response):
        """
        Handle incoming service requests.
        
        Args:
            request: Contains x, y, theta fields (we'll only use theta)
            response: Empty response (TeleportAbsolute has no response fields)
        
        Returns:
            response: The service response
        """
        desired_angle = request.theta
        
        self.get_logger().info(
            f'Received request to rotate to {math.degrees(desired_angle):.2f} degrees'
        )
        
        # Create a request for the teleport service
        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = self.current_pose.x  # Keep current X position
        teleport_request.y = self.current_pose.y  # Keep current Y position
        teleport_request.theta = desired_angle     # Set new angle
        
        # Call the teleport service
        future = self.teleport_client.call_async(teleport_request)
        
        self.get_logger().info(f'Rotated turtle to {math.degrees(desired_angle):.2f} degrees')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    rotate_service = RotateService()
    rclpy.spin(rotate_service)
    rotate_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 6.3: Make executable and update setup.py

```bash
chmod +x rotate_service.py
cd ~/turtle_ws/src/turtle_controller
nano setup.py
```

Update entry_points:

```python
entry_points={
    'console_scripts': [
        'velocity_publisher = turtle_controller.velocity_publisher:main',
        'pose_subscriber = turtle_controller.pose_subscriber:main',
        'rotate_service = turtle_controller.rotate_service:main',
    ],
},
```

### Step 6.4: Build and test

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

**Test the service:**

Terminal 1 - Turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2 - Your service server:
```bash
ros2 run turtle_controller rotate_service
```

Terminal 3 - Call the service manually:
```bash
# Rotate to 90 degrees (pi/2 radians)
ros2 service call /rotate_turtle turtlesim/srv/TeleportAbsolute "{x: 0.0, y: 0.0, theta: 1.57}"

# Rotate to 180 degrees (pi radians)
ros2 service call /rotate_turtle turtlesim/srv/TeleportAbsolute "{x: 0.0, y: 0.0, theta: 3.14}"
```

---

## 7. Creating a Service Client

A **service client** calls services. We'll create a client that requests our rotation service.

### Step 7.1: Create the service client script

```bash
cd ~/turtle_ws/src/turtle_controller/turtle_controller
nano rotate_client.py
```

### Step 7.2: Write the service client code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
import math
import sys


class RotateClient(Node):
    """
    A ROS 2 service client that calls the rotate service.
    """
    
    def __init__(self):
        super().__init__('rotate_client')
        
        # Create a client for our rotate service
        self.client = self.create_client(TeleportAbsolute, '/rotate_turtle')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /rotate_turtle service...')
        
        self.get_logger().info('Connected to rotate service!')
    
    def send_request(self, angle_degrees):
        """
        Send a rotation request to the service.
        
        Args:
            angle_degrees (float): The desired angle in degrees
        """
        # Create a service request
        request = TeleportAbsolute.Request()
        request.x = 0.0  # Not used by our service, but required
        request.y = 0.0  # Not used by our service, but required
        request.theta = math.radians(angle_degrees)  # Convert degrees to radians
        
        self.get_logger().info(f'Sending request to rotate to {angle_degrees} degrees...')
        
        # Call the service asynchronously
        future = self.client.call_async(request)
        
        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Successfully rotated to {angle_degrees} degrees!')
        else:
            self.get_logger().error('Service call failed!')


def main(args=None):
    rclpy.init(args=args)
    
    # Get the desired angle from command line arguments
    if len(sys.argv) < 2:
        print('Usage: ros2 run turtle_controller rotate_client <angle_in_degrees>')
        print('Example: ros2 run turtle_controller rotate_client 90')
        return
    
    try:
        angle = float(sys.argv[1])
    except ValueError:
        print('Error: Please provide a valid number for the angle')
        return
    
    # Create the client and send the request
    rotate_client = RotateClient()
    rotate_client.send_request(angle)
    
    rotate_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 7.3: Make executable and update setup.py

```bash
chmod +x rotate_client.py
cd ~/turtle_ws/src/turtle_controller
nano setup.py
```

Update entry_points:

```python
entry_points={
    'console_scripts': [
        'velocity_publisher = turtle_controller.velocity_publisher:main',
        'pose_subscriber = turtle_controller.pose_subscriber:main',
        'rotate_service = turtle_controller.rotate_service:main',
        'rotate_client = turtle_controller.rotate_client:main',
    ],
},
```

### Step 7.4: Build and test

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

**Test the client:**

Terminal 1 - Turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

Terminal 2 - Service server:
```bash
ros2 run turtle_controller rotate_service
```

Terminal 3 - Service client:
```bash
# Rotate to 45 degrees
ros2 run turtle_controller rotate_client 45

# Rotate to 180 degrees
ros2 run turtle_controller rotate_client 180

# Rotate to 270 degrees
ros2 run turtle_controller rotate_client 270
```

---

## 8. Testing Everything Together

Now let's run all components together to see the complete system in action!

### Complete System Test

**Terminal 1** - Start turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2** - Start the pose subscriber:
```bash
ros2 run turtle_controller pose_subscriber
```

**Terminal 3** - Start the rotate service:
```bash
ros2 run turtle_controller rotate_service
```

**Terminal 4** - Start the velocity publisher (turtle will start moving):
```bash
ros2 run turtle_controller velocity_publisher
```

**Terminal 5** - Use the rotate client:
```bash
# Try different angles while the turtle is moving!
ros2 run turtle_controller rotate_client 90
ros2 run turtle_controller rotate_client 180
ros2 run turtle_controller rotate_client 0
```

### Useful ROS 2 Commands for Debugging

**List all active nodes:**
```bash
ros2 node list
```

**List all active topics:**
```bash
ros2 topic list
```

**See information about a specific topic:**
```bash
ros2 topic info /turtle1/cmd_vel
ros2 topic echo /turtle1/pose
```

**List all services:**
```bash
ros2 service list
```

**See information about a service:**
```bash
ros2 service type /rotate_turtle
```

**View the computational graph:**
```bash
rqt_graph
```

---

## Summary

Congratulations! You've learned:

1. **Workspace Creation** - Organized structure for ROS 2 projects
2. **Package Creation** - Built a Python package with dependencies
3. **Building & Installing** - Used colcon to build your package
4. **Topic Publisher** - Created a node that publishes velocity commands
5. **Topic Subscriber** - Created a node that listens to pose information
6. **Service Server** - Created a service that rotates the turtle
7. **Service Client** - Created a client that calls the rotation service

### Key Concepts Recap

- **Nodes**: Independent processes that perform computation (all our scripts are nodes)
- **Topics**: Named channels for streaming data (publish/subscribe pattern)
- **Services**: Request/response communication (like function calls over the network)
- **Messages**: Data structures sent over topics
- **Publishers**: Send messages to topics
- **Subscribers**: Receive messages from topics
- **Service Servers**: Provide services that can be called
- **Service Clients**: Call services and receive responses

### Next Steps

- Try modifying the velocity publisher to make different movement patterns
- Create a service that moves the turtle to specific coordinates
- Combine multiple services to create complex turtle behaviors
- Explore creating custom message and service types

### Common Issues and Solutions

**Issue**: "Package not found" after building
- **Solution**: Make sure you've sourced the workspace: `source ~/turtle_ws/install/setup.bash`

**Issue**: "No executable found"
- **Solution**: Check that you've added the entry point in `setup.py` and rebuilt the package

**Issue**: Service or topic not found
- **Solution**: Make sure turtlesim is running and use `ros2 topic list` or `ros2 service list` to verify

**Issue**: Permission denied when running scripts
- **Solution**: Make sure you made the scripts executable with `chmod +x script_name.py`

---

## 9. Creating a Launch File

**Launch files** allow you to start multiple nodes with a single command. Instead of opening 5 different terminals, you can launch everything at once!

### What is a Launch File?

A launch file is a Python script (in ROS 2) that describes:
- Which nodes to start
- What parameters to set
- What arguments to pass
- Remapping of topics/services
- Conditional logic for different scenarios

### Step 9.1: Create the launch directory

```bash
cd ~/turtle_ws/src/turtle_controller
mkdir launch
cd launch
```

### Step 9.2: Create a simple launch file

```bash
nano turtle_system_launch.py
```

### Step 9.3: Write the launch file code

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to start the complete turtle control system.
    
    This will start:
    1. Turtlesim node
    2. Velocity publisher (makes turtle move)
    3. Pose subscriber (prints turtle position)
    4. Rotate service (provides rotation service)
    """
    
    return LaunchDescription([
        # Start the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Start our velocity publisher
        Node(
            package='turtle_controller',
            executable='velocity_publisher',
            name='velocity_publisher',
            output='screen'
        ),
        
        # Start our pose subscriber
        Node(
            package='turtle_controller',
            executable='pose_subscriber',
            name='pose_subscriber',
            output='screen'
        ),
        
        # Start our rotate service
        Node(
            package='turtle_controller',
            executable='rotate_service',
            name='rotate_service',
            output='screen'
        ),
    ])
```

**Explanation of parameters:**
- `package`: The ROS 2 package containing the executable
- `executable`: The name of the node executable (from setup.py entry_points)
- `name`: The name to give this node instance (can override the node's internal name)
- `output='screen'`: Print node output to the terminal (instead of log files)

### Step 9.4: Update setup.py to include the launch file

```bash
cd ~/turtle_ws/src/turtle_controller
nano setup.py
```

Add the following import at the top:

```python
from setuptools import setup
import os
from glob import glob
```

Then modify the setup function to include the launch files (find the `data_files` section):

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Include launch files
    (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
],
```

**Complete setup.py should look like this:**

```python
from setuptools import setup
import os
from glob import glob

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Turtle controller package for ROS 2 tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = turtle_controller.velocity_publisher:main',
            'pose_subscriber = turtle_controller.pose_subscriber:main',
            'rotate_service = turtle_controller.rotate_service:main',
            'rotate_client = turtle_controller.rotate_client:main',
        ],
    },
)
```

### Step 9.5: Build and source the workspace

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

### Step 9.6: Run the launch file

```bash
ros2 launch turtle_controller turtle_system_launch.py
```

You should see:
- The turtlesim window open
- The turtle start moving in a circle
- Pose information printing to the terminal
- All nodes running together!

To stop everything, press **Ctrl+C** once - it will shut down all nodes.

### Step 9.7: Test the service while the system is running

Open a new terminal:

```bash
source ~/turtle_ws/install/setup.bash
ros2 run turtle_controller rotate_client 90
```

The turtle will rotate while continuing to move!

---

## Launch File Best Practices

### 1. Naming Convention
- Use descriptive names: `robot_bringup.launch.py`, `navigation_launch.py`
- End with `.launch.py` for clarity

### 2. Organization
```python
LaunchDescription([
    # 1. Declare arguments first
    DeclareLaunchArgument(...),
    
    # 2. Log information
    LogInfo(...),
    
    # 3. Launch nodes in logical order
    Node(...),  # Core nodes first
    Node(...),  # Supporting nodes next
])
```

### 3. Use Parameters
You can pass parameters to nodes:

```python
Node(
    package='turtle_controller',
    executable='velocity_publisher',
    name='velocity_publisher',
    parameters=[
        {'linear_speed': 2.0},
        {'angular_speed': 1.0}
    ]
)
```

### 4. Topic Remapping
Remap topics to different names:

```python
Node(
    package='turtle_controller',
    executable='velocity_publisher',
    name='velocity_publisher',
    remappings=[
        ('/turtle1/cmd_vel', '/my_turtle/cmd_vel')
    ]
)
```

---

## Useful Launch Commands

**List available launch files in a package:**
```bash
ros2 launch turtle_controller <TAB><TAB>
```

**See launch file arguments:**
```bash
ros2 launch turtle_controller turtle_advanced_launch.py --show-args
```

**Run with verbose output:**
```bash
ros2 launch turtle_controller turtle_system_launch.py --debug
```

---

## Complete Launch File Example with Everything

Here's a final comprehensive example combining all concepts:

```bash
cd ~/turtle_ws/src/turtle_controller/launch
nano turtle_complete_launch.py
```

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Complete turtle system launch file with all features.
    """
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='2.0',
        description='Linear speed for turtle movement'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Angular speed for turtle rotation'
    )
    
    enable_movement_arg = DeclareLaunchArgument(
        'enable_movement',
        default_value='true',
        description='Enable automatic turtle movement'
    )
    
    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_movement = LaunchConfiguration('enable_movement')
    
    # Define nodes
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    velocity_publisher_node = Node(
        package='turtle_controller',
        executable='velocity_publisher',
        name='velocity_publisher',
        output='screen',
        condition=IfCondition(enable_movement)
    )
    
    pose_subscriber_node = Node(
        package='turtle_controller',
        executable='pose_subscriber',
        name='pose_subscriber',
        output='screen'
    )
    
    rotate_service_node = Node(
        package='turtle_controller',
        executable='rotate_service',
        name='rotate_service',
        output='screen'
    )
    
    # Add a delayed start for velocity publisher (optional)
    # This gives time for turtlesim to initialize
    delayed_velocity_publisher = TimerAction(
        period=2.0,
        actions=[velocity_publisher_node]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        linear_speed_arg,
        angular_speed_arg,
        enable_movement_arg,
        
        # Startup message
        LogInfo(msg=['='*50]),
        LogInfo(msg=['Starting Complete Turtle Control System']),
        LogInfo(msg=['='*50]),
        
        # Launch nodes
        turtlesim_node,
        pose_subscriber_node,
        rotate_service_node,
        delayed_velocity_publisher,  # Starts after 2 seconds
        
        # Completion message
        LogInfo(msg=['All nodes launched successfully!']),
    ])
```

Build and test:

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash

# Test with different configurations
ros2 launch turtle_controller turtle_complete_launch.py

ros2 launch turtle_controller turtle_complete_launch.py enable_movement:=false

ros2 launch turtle_controller turtle_complete_launch.py linear_speed:=5.0 angular_speed:=2.0
```

---

## Summary of Launch Files

You've now learned:

1. **Basic Launch Files** - Starting multiple nodes with one command
2. **Launch Arguments** - Making launch files configurable
3. **Conditional Execution** - Enabling/disabling nodes dynamically
4. **Parameters** - Passing configuration to nodes
5. **Topic Remapping** - Changing topic names at launch time
6. **Delayed Actions** - Starting nodes with delays
7. **Logging** - Adding informative messages

### Why Use Launch Files?

- **Convenience**: Start entire systems with one command
- **Reproducibility**: Same configuration every time
- **Flexibility**: Easy to modify behavior with arguments
- **Professional**: Industry standard for ROS 2 projects
- **Debugging**: Centralized control of all nodes

### Next Steps with Launch Files

- Include other launch files (composability)
- Use YAML configuration files for parameters
- Add lifecycle nodes management
- Create launch files for different robot configurations
- Learn about launch testing

Happy ROS 2 learning!
