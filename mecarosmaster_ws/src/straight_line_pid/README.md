# straight_line_pid

PID controller for perfectly straight-line motion on the **Rosmaster X3**,
built with `control_toolbox::PidROS` and standard ROS2 Humble features.

## How it works

1. When you publish a non-zero `linear.x` to `/cmd_vel_straight`, the node
   **locks the current IMU yaw** as the reference heading.
2. On every control tick (default 50 Hz) it computes:
   ```
   error     = target_yaw − current_yaw   (wrapped to [-π, π])
   angular.z = PID.computeCommand(error, dt)
   ```
3. It publishes `{ linear.x: your_speed, angular.z: correction }` directly
   to `/diff_drive_controller/cmd_vel`.
4. Publishing `linear.x = 0` releases the lock and sends a stop.

## Dependencies (Humble)

```bash
sudo apt install \
  ros-humble-control-toolbox \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs
```

## Build

```bash
cd ~/ros2_ws
# Copy or symlink the package
cp -r /path/to/straight_line_pid src/

colcon build --packages-select straight_line_pid --symlink-install
source install/setup.bash
```

## Run

```bash
# Default gains from config/pid_params.yaml
ros2 launch straight_line_pid straight_line_pid.launch.py

# Override gains from CLI
ros2 launch straight_line_pid straight_line_pid.launch.py kp:=1.5 kd:=0.2

# Send a forward command (0.3 m/s)
ros2 topic pub /cmd_vel_straight geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"

# Send a backward command (-0.2 m/s)
ros2 topic pub /cmd_vel_straight geometry_msgs/msg/Twist \
  "{linear: {x: -0.2}, angular: {z: 0.0}}"

# Stop
ros2 topic pub --once /cmd_vel_straight geometry_msgs/msg/Twist "{}"
```

## Live tuning (no restart)

```bash
ros2 param set /straight_line_pid pid.p 1.5
ros2 param set /straight_line_pid pid.i 0.005
ros2 param set /straight_line_pid pid.d 0.15
```

## Monitor

```bash
# Watch corrected output
ros2 topic echo /diff_drive_controller/cmd_vel

# Watch PID internals (debug logs must be enabled)
ros2 run rqt_plot rqt_plot
# → add /diff_drive_controller/cmd_vel/angular/z
```

## Tuning guide

| Symptom                          | Fix                         |
|----------------------------------|-----------------------------|
| Robot drifts but slowly corrects | Increase `Kp`               |
| Robot oscillates left/right      | Decrease `Kp`, increase `Kd`|
| Slow steady-state offset remains | Add small `Ki` (0.005–0.02) |
| Integral windup / growing wobble | Decrease `Ki`, check `i_clamp` |

Start with `Ki = 0`, `Kd = 0` and tune `Kp` alone first.

## Topics summary

| Topic | Direction | Type | Role |
|---|---|---|---|
| `/cmd_vel_straight` | IN | Twist | User command (linear.x only) |
| `/mecarosmaster/imu/data` | IN | Imu | Yaw feedback |
| `/diff_drive_controller/cmd_vel` | OUT | Twist | Corrected command |
