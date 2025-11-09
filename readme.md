# Turtle Controller - ROS 2 Humble Tutorial Package

A comprehensive beginner-friendly ROS 2 package demonstrating fundamental concepts using the turtlesim simulator. This package includes examples of publishers, subscribers, services, clients, and launch files.

## Overview

This package is designed as a complete learning resource for ROS 2 beginners. It demonstrates:

- **Topic Publishers** - Sending velocity commands to control turtle movement
- **Topic Subscribers** - Receiving and displaying turtle pose information
- **Service Servers** - Providing rotation services
- **Service Clients** - Calling rotation services
- **Launch Files** - Starting multiple nodes simultaneously

<br />
<br />

## Learning Objectives

By working through this package, you will learn:

- How to create and structure a ROS 2 workspace
- How to create a Python-based ROS 2 package
- How to implement publishers and subscribers
- How to create and use services
- How to write launch files for complex systems
- ROS 2 best practices and conventions

<br />
<br />

## Prerequisites

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble installed and configured
- Python 3.10+
- Basic understanding of Python programming
- Terminal/command line familiarity

<br />
<br />

### Installation Check

Verify your ROS 2 installation:

```bash
ros2 --version
```

You should see output similar to: `ros2 cli version: 0.18.x`

<br />
<br />

## Learning Outcomes

After completing these exercises, you should be able to:

- Create ROS 2 workspaces and packages  
- Write publisher nodes that send messages  
- Write subscriber nodes that receive messages  
- Create service servers that handle requests  
- Create service clients that call services  
- Write launch files to manage multiple nodes  
- Use ROS 2 command-line tools for debugging  
- Understand the publish-subscribe pattern  
- Understand the request-response pattern

<br />
<br />

## Exercises

1. Exercise 1: [Setting Up Your Workspace](./exercise_1.md)
2. Exercise 2: [Creating a Topic Publisher](exercise_2.md)
3. Exercise 3: [Creating a Topic Subscriber](exercise_3.md)
4. Exercise 4: [Creating a Service Server](./exercise_4.md)
5. Exercise 5: [Creating a Service Client](./exercise_5.md)
6. Exercise 6: [Creating Launch Files](./exercise_6.md)

<br />
<br />

## Verification Checklist

Use this checklist to verify you've completed everything:

- [ ] Workspace created and built successfully
- [ ] Package created with correct dependencies
- [ ] Velocity publisher moves turtle in a circle
- [ ] Pose subscriber prints turtle position
- [ ] Rotate service can be called manually
- [ ] Rotate client works from command line
- [ ] Basic launch file starts all nodes
- [ ] All nodes appear in `ros2 node list`
- [ ] All topics appear in `ros2 topic list`

<br />
<br />

## Next Steps

Once you've completed all exercises:

1. **Experiment**: Modify the velocity values to create different movement patterns
2. **Extend**: Add new features (e.g., move to specific coordinates)
3. **Optimize**: Can you reduce code duplication?
4. **Document**: Add detailed comments explaining your code
5. **Share**: Help others who are stuck!

<br />
<br />

## Final Package Structure

```
turtle_controller/
├── launch/
│   ├── turtle_system_launch.py          # Basic launch file
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

<br />
<br />

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

