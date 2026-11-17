# cestra
A collection of random robotics ideas and the barebones code to back them up.

## `cestra_turtle` development environment

The Pixi environment provides ROS 2 Jazzy, TurtleBot3, Gazebo integration,
RViz, OpenCV, and the tools needed to build `cestra_turtle`. It is locked for
Linux x86-64, Intel and Apple Silicon macOS, and Windows x86-64.

Install Pixi, then from the repository root run:

```sh
pixi install
pixi run build
```

To use the locally built package, activate the colcon overlay before launching:

```sh
# Linux and macOS
source install/setup.bash
ros2 launch cestra_turtle line_follow.launch.py
```

On Windows, activate `install\setup.ps1` in PowerShell (or
`install\setup.bat` in Command Prompt) before running the same `ros2 launch`
command. The Pixi environment only builds `cestra_turtle`; it does not attempt
to build the other rough packages in this repository.
