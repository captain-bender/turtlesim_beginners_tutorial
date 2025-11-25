## Exercise 1: Setting Up Your Workspace

### Goal
Create a ROS 2 workspace and package from scratch.

### Tasks

**Task 1.1:** Create a workspace directory structure
- Create a directory called `turtle_ws` in your home directory
- Inside it, create a `src` subdirectory
- Navigate to the workspace root

<details>
<summary> Hint 1.1</summary>

Use the `mkdir` command with the `-p` flag to create nested directories.
```bash
mkdir -p ~/directory_name/subdirectory_name
```
</details>

<details>
<summary> Check Your Work 1.1</summary>

Run `pwd` - you should be in `/home/your_username/turtle_ws`
Run `ls` - you should see a `src` directory
</details>

**Task 1.2:** Create a new ROS 2 Python package
- Navigate to the `src` directory
- Create a package named `turtle_controller`
- The package should depend on: `rclpy`, `geometry_msgs`, and `turtlesim`

<details>
<summary> Hint 1.2</summary>

Use the `ros2 pkg create` command with these flags:
- `--build-type ament_python` for Python packages
- `--dependencies` to list dependencies
</details>

<details>
<summary> More Help 1.2</summary>

```bash
ros2 pkg create --build-type ament_python PACKAGE_NAME \
  --dependencies dependency1 dependency2 dependency3
```
</details>

<details>
<summary> Check Your Work 1.2</summary>

- Navigate to `~/turtle_ws/src/turtle_controller`
- You should see: `package.xml`, `setup.py`, `setup.cfg`, `resource/`, `test/`, and a `turtle_controller/` directory
</details>

**Task 1.3:** Build your workspace
- Navigate back to the workspace root
- Build the package using colcon
- Source the installation

<details>
<summary> Hint 1.3</summary>

The build command is `colcon build`
After building, source the file: `install/setup.bash`
</details>

<details>
<summary> Check Your Work 1.3</summary>

Run: `ros2 pkg list | grep turtle_controller`
You should see `turtle_controller` in the output.
</details>
