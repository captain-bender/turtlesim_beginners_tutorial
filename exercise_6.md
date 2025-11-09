## Exercise 6: Creating Launch Files

### Goal
Create launch files to start multiple nodes with a single command.

### Background Information

**Key Concepts:**
- **Launch files** start multiple nodes at once
- Written in Python for ROS 2
- Located in a `launch/` directory
- Must be registered in setup.py

### Tasks

**Task 6.1:** Create launch directory
- Navigate to your package root (`~/turtle_ws/src/turtle_controller`)
- Create a `launch` directory

**Task 6.2:** Create a basic launch file

Create `turtle_system_launch.py` in the launch directory that:
1. Imports necessary modules
2. Defines a `generate_launch_description()` function
3. Returns a `LaunchDescription` with multiple Node actions

<details>
<summary> Hint 6.2 - Imports</summary>

```python
from launch import LaunchDescription
from launch_ros.actions import Node
```
</details>

<details>
<summary> Hint 6.2 - Structure</summary>

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='node_name',
            name='node_name',
            output='screen'
        ),
        # More nodes here...
    ])
```
</details>

**Task 6.3:** Add nodes to launch

Add Node actions for:
1. turtlesim_node (from turtlesim package)
2. velocity_publisher
3. pose_subscriber
4. rotate_service

<details>
<summary> Hint 6.3</summary>

For each node, specify:
- `package`: The package containing the executable
- `executable`: The name from setup.py entry_points
- `name`: A name for this node instance
- `output='screen'`: Print to terminal
</details>

**Task 6.4:** Update setup.py to include launch files

Modify setup.py to install launch files:
1. Add imports at the top
2. Add launch files to `data_files`

<details>
<summary> Hint 6.4 - Imports</summary>

```python
import os
from glob import glob
```
</details>

<details>
<summary> Hint 6.4 - data_files</summary>

Add this line to the `data_files` list:
```python
(os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
```
</details>

**Task 6.5:** Build and test the launch file

<details>
<summary> Hint 6.5</summary>

```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
ros2 launch turtle_controller turtle_system_launch.py
```

Everything should start at once!
</details>

**Challenge Task 6.6:** Create an advanced launch file (Optional)

Create `turtle_advanced_launch.py` that:
- Accepts arguments (enable_publisher, enable_subscriber)
- Conditionally starts nodes based on arguments
- Includes log messages

<details>
<summary> Hint 6.6</summary>

You'll need additional imports:
```python
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
```

Example of conditional node:
```python
Node(
    package='turtle_controller',
    executable='velocity_publisher',
    condition=IfCondition(LaunchConfiguration('enable_publisher'))
)
```
</details>