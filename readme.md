# Turtle Controller - ROS 2 Humble Tutorial Package

A comprehensive beginner-friendly ROS 2 package demonstrating fundamental concepts using the turtlesim simulator. This package includes examples of publishers, subscribers, services, clients, and launch files.

## Overview

This package is designed as a complete learning resource for ROS 2 beginners. It demonstrates:

- **Topic Publishers** - Sending velocity commands to control turtle movement
- **Topic Subscribers** - Receiving and displaying turtle pose information
- **Service Servers** - Providing rotation services
- **Service Clients** - Calling rotation services
- **Launch Files** - Starting multiple nodes simultaneously

## Learning Objectives

By working through this package, you will learn:

- How to create and structure a ROS 2 workspace
- How to create a Python-based ROS 2 package
- How to implement publishers and subscribers
- How to create and use services
- How to write launch files for complex systems
- ROS 2 best practices and conventions

## Prerequisites

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble installed and configured
- Python 3.10+
- Basic understanding of Python programming
- Terminal/command line familiarity

### Installation Check

Verify your ROS 2 installation:

```bash
ros2 --version
```

You should see output similar to: `ros2 cli version: 0.18.x`

## Package Structure

```
turtle_controller/
├── launch/
│   ├── turtle_system_launch.py          # Basic launch file
│   ├── turtle_advanced_launch.py        # Launch file with arguments
│   └── turtle_complete_launch.py        # Comprehensive launch file
├── turtle_controller/
│   ├── __init__.py
│   ├── velocity_publisher.py            # Publishes velocity commands
│   ├── pose_subscriber.py               # Subscribes to turtle pose
│   ├── rotate_service.py                # Service server for rotation
│   └── rotate_client.py                 # Service client for rotation
├── package.xml                          # Package metadata
├── setup.py                             # Python package setup
├── setup.cfg                            # Package configuration
└── README.md                            # This file
```


## Nodes Description

### 1. Velocity Publisher (`velocity_publisher`)

Publishes `Twist` messages to `/turtle1/cmd_vel` to make the turtle move in a circular pattern.

**Run standalone:**
```bash
ros2 run turtle_controller velocity_publisher
```

**Topics Published:**
- `/turtle1/cmd_vel` (geometry_msgs/msg/Twist) - Velocity commands

### 2. Pose Subscriber (`pose_subscriber`)

Subscribes to `/turtle1/pose` and displays the turtle's position and orientation.

**Run standalone:**
```bash
ros2 run turtle_controller pose_subscriber
```

**Topics Subscribed:**
- `/turtle1/pose` (turtlesim/msg/Pose) - Turtle position and orientation

### 3. Rotate Service (`rotate_service`)

Provides a service to rotate the turtle to a specific angle while maintaining its current position.

**Run standalone:**
```bash
ros2 run turtle_controller rotate_service
```

**Services Provided:**
- `/rotate_turtle` (turtlesim/srv/TeleportAbsolute) - Rotate to specified angle

**Call the service manually:**
```bash
# Rotate to 90 degrees (1.57 radians)
ros2 service call /rotate_turtle turtlesim/srv/TeleportAbsolute "{x: 0.0, y: 0.0, theta: 1.57}"
```

### 4. Rotate Client (`rotate_client`)

A command-line client that calls the rotation service with a specified angle in degrees.

**Usage:**
```bash
ros2 run turtle_controller rotate_client <angle_in_degrees>
```

**Examples:**
```bash
ros2 run turtle_controller rotate_client 45
ros2 run turtle_controller rotate_client 90
ros2 run turtle_controller rotate_client 180
```

## Usage Examples

### Example 1: Manual Turtle Control with Pose Monitoring

**Terminal 1** - Start turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2** - Monitor pose:
```bash
ros2 run turtle_controller pose_subscriber
```

**Terminal 3** - Control with keyboard:
```bash
ros2 run turtlesim turtle_teleop_key
```

### Example 2: Automatic Movement with Service Control

**Terminal 1** - Launch system:
```bash
ros2 launch turtle_controller turtle_system_launch.py
```

**Terminal 2** - Change turtle direction:
```bash
ros2 run turtle_controller rotate_client 0
ros2 run turtle_controller rotate_client 90
ros2 run turtle_controller rotate_client 180
```

### Example 3: Advanced Launch with Arguments

**Launch with movement disabled:**
```bash
ros2 launch turtle_controller turtle_advanced_launch.py enable_publisher:=false
```

**Launch without pose subscriber:**
```bash
ros2 launch turtle_controller turtle_advanced_launch.py enable_subscriber:=false
```

**Launch with only turtlesim and service:**
```bash
ros2 launch turtle_controller turtle_advanced_launch.py enable_publisher:=false enable_subscriber:=false
```

## Useful Commands

### Inspect Running System

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# See topic info
ros2 topic info /turtle1/cmd_vel

# Echo topic messages
ros2 topic echo /turtle1/pose

# List services
ros2 service list

# See service type
ros2 service type /rotate_turtle

# View computational graph
rqt_graph
```

### Debugging

```bash
# Run with debug output
ros2 run turtle_controller velocity_publisher --ros-args --log-level debug

# Check node info
ros2 node info /velocity_publisher

# Monitor topic frequency
ros2 topic hz /turtle1/cmd_vel
```

## Additional Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Turtlesim Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

### Learning Materials
- [Understanding ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Understanding Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Understanding Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Creating Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)

