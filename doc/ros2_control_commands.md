# ros2_control commands

## List controllers
```bash
ros2 control list_controllers
```

## List hardware interfaces
```bash
ros2 control list_hardware_interfaces
```

## Activate a controller
```bash
ros2 control set_controller_state <controller_name> active
```

## Deactivate a controller
```bash
ros2 control set_controller_state <controller_name> inactive
```

## Switch controllers
```bash
ros2 control switch_controllers --activate <controller_to_activate> --deactivate <controller_to_deactivate>
```

## Load a controller (spawner)
```bash
ros2 run controller_manager spawner <controller_name>
```

## Load a controller inactive (spawner)
```bash
ros2 run controller_manager spawner <controller_name> --inactive
```

## Unload a controller
```bash
ros2 control unload_controller <controller_name>
```

---

## Manipulator

### Check controllers
```bash
ros2 control list_controllers
```

### Activate
```bash
ros2 control set_controller_state joint_trajectory_controller active
```

### Command joints
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [joint1, joint2], points: [{positions: [0.3, 0.5], time_from_start: {sec: 2}}]}" -1
```

---

## Ackermann

### Check controllers
```bash
ros2 control list_controllers
```

### Activate
```bash
ros2 control set_controller_state ackermann_steering_controller active
```

### Command drive/steering
```bash
ros2 topic pub /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.2}}}"
```

### Check odom TF
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
