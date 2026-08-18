# Turtlesim+ interfaces

What the `turtlesim_plus` node offers: 7 topics, 12 services, 1 action.
Source: `turtlesim_plus_ws/src/turtlesim_plus/turtlesim_plus/turtlesim_plus/ros2_plugins.py`.
`[name]` is a turtle name, for example `turtle1`.

## Topics

| Name | Type | Direction | What it does |
|---|---|---|---|
| `/mouse_position` | `geometry_msgs/Point` | node publishes | Where you clicked in the window, in world units. |
| `/[name]/pose` | `turtlesim/Pose` | node publishes | The turtle's position x, y and heading θ. |
| `/[name]/scan` | `turtlesim_plus_interfaces/ScannerDataArray` | node publishes | Things seen in front of the turtle: type, angle, distance. |
| `/[name]/pizza_count` | `std_msgs/Int64` | node publishes | How many pizzas this turtle has eaten. |
| `/[name]/parcel_count` | `std_msgs/Int64` | node publishes | How many parcels this turtle has delivered. |
| `/[name]/carrying_parcel` | `std_msgs/Bool` | node publishes | True while the turtle holds a parcel. |
| `/[name]/cmd_vel` | `geometry_msgs/Twist` | node listens | Drive the turtle: `linear.x` forward speed, `angular.z` turn rate. |

## Services

| Name | Type | What it does |
|---|---|---|
| `/spawn_turtle` | `turtlesim/Spawn` | Add a turtle at x, y, θ. Empty or duplicate name gets renamed for you. |
| `/remove_turtle` | `turtlesim/Kill` | Delete the turtle with that name. |
| `/spawn_pizza` | `turtlesim_plus_interfaces/GivePosition` | Put a pizza at x, y. |
| `/spawn_parcel` | `turtlesim_plus_interfaces/GivePosition` | Put a parcel at x, y. |
| `/clear` | `std_srvs/Empty` | Erase the drawn trails of every turtle. |
| `/[name]/stop` | `std_srvs/Empty` | Set that turtle's speed to zero. |
| `/[name]/set_pen` | `turtlesim/SetPen` | Change trail colour r, g, b, width, or turn the trail off. |
| `/[name]/teleport_absolute` | `turtlesim/TeleportAbsolute` | Jump the turtle to x, y, θ in world coordinates. |
| `/[name]/teleport_relative` | `turtlesim/TeleportRelative` | Jump the turtle forward and turn, measured from where it is now. |
| `/[name]/eat` | `std_srvs/Empty` | Eat one pizza inside the eat range, and pizza_count goes up by one. |
| `/[name]/pickup` | `std_srvs/Empty` | Pick up one parcel inside the pickup range, if not already carrying one. |
| `/[name]/dropoff` | `std_srvs/Empty` | Drop the carried parcel. It only counts inside the drop-off zone. |

## Action

| Name | Type | What it does |
|---|---|---|
| `/[name]/detect_pizza` | `turtlesim_plus_interfaces/GetData` | Returns the pizzas the scanner currently sees. Aborts if it sees nothing. |

## Notes

- Nothing per-turtle exists until a turtle is spawned. `turtle1` is spawned at start-up.
- A pizza or parcel spawn with x or y as NaN lands at a random spot.
- Scanner range, eat range and pickup range are node parameters: `scanner_radius`, `scanner_angle_range`, `eat_radius`, `eat_angle_range`, `pickup_radius`, `pickup_angle_range`, plus `time_step`.

## The colored wedges

Each turtle draws three wedges. Change their size with node parameters.

| Color | Meaning | Radius parameter | Angle parameter |
|---|---|---|---|
| Red | Scanner, what `/[name]/scan` reports | `scanner_radius`, default 4.0 | `scanner_angle_range`, default pi/3 |
| Green | Eat range, where `/[name]/eat` works | `eat_radius`, default 2.0 | `eat_angle_range`, default pi/3 |
| Blue | Pickup range, where `/[name]/pickup` works | `pickup_radius`, default 2.0 | `pickup_angle_range`, default pi/3 |

An angle of `6.283` (2 pi) makes that wedge a full circle.

```bash
ros2 run turtlesim_plus turtlesim_plus_node.py --ros-args -p scanner_radius:=1.0 -p eat_radius:=0.5 -p pickup_radius:=3.0
```
