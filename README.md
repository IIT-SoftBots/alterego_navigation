# alterego_navigation (ROS2 / Nav2 Migration)

Helper nodes and services for AlterEGO navigation, migrated from ROS1 (move_base) to ROS2 (Nav2 + NavigateToPose action).

## Provided Nodes (Python)
Installed in `lib/alterego_navigation`:
* `nav2points.py` (WIP) – listens to `target_location` and will send a `NavigateToPose` goal (location resolution placeholder).
* `nav2points_service.py` – exposes `navigation_service` (`alterego_msgs/NavService`) to trigger a navigation goal via Nav2.
* `initialpose.py` – publishes an initial pose using parameter `navigation_locations`.
* `send_key_points.py` – publishes nearest key point on `nearest_point` and RViz markers (`keypoint_markers_array`).
* `where_are_you_service.py` – service `where_are_you_service` (`alterego_msgs/WhereRUService`) returning nearest key point.

## Parameters (expected)
You must supply structured parameters (converted from old ROS1 parameter server layout):
* `navigation_locations_hier`: list of dicts: `[{LocationName: [ {SubLocationName: {position: {x:..}, orientation: {x:..}}}, ... ]}, ...]`
* `navigation_keypoints`: list of dicts: `[{KeyPointName: {position: {...}, orientation: {...}}}, ...]`
* `navigation_locations`: simplified list of dicts for initial pose (contains an entry with key `Ingresso`).

Because ROS2 parameters do not natively support arbitrary nested YAML with lists-of-dicts in all launch styles, it is recommended to load them from a YAML file in a launch description.

Example YAML snippet:
```yaml
alterego_navigation:
  ros__parameters:
	navigation_locations_hier: []        # Fill with your structured locations
	navigation_keypoints: []             # Fill with keypoints
	navigation_locations: []             # Must contain an entry with 'Ingresso'
```

## Launch (example skeleton)
Create a launch file (e.g. `launch/navigation_nodes.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	params_file = '/path/to/your/navigation_params.yaml'
	return LaunchDescription([
		Node(
			package='alterego_navigation',
			executable='initialpose.py',
			name='initialpose_publisher',
			parameters=[params_file]
		),
		Node(
			package='alterego_navigation',
			executable='nav2points_service.py',
			name='nav_service',
			parameters=[params_file]
		),
		Node(
			package='alterego_navigation',
			executable='send_key_points.py',
			name='keypoints',
			parameters=[params_file]
		),
		Node(
			package='alterego_navigation',
			executable='where_are_you_service.py',
			name='where_are_you',
			parameters=[params_file]
		),
	])
```

Start Nav2 separately (map_server, amcl, lifecycle manager, etc.) ensuring the `navigate_to_pose` action server is available.

## Topics / Services
* Action: `navigate_to_pose` (Nav2 standard)
* Service: `/navigation_service` (alterego_msgs/NavService)
* Service: `/where_are_you_service` (alterego_msgs/WhereRUService)
* Topic (pub): `goal_reached` (`std_msgs/String`) – basic success notification
* Topic (sub): `target_location` (`std_msgs/String`) – target name (resolution pending TODO)
* Topic (pub): `nearest_point` (`std_msgs/String`)
* Topic (pub): `keypoint_markers_array` (`visualization_msgs/MarkerArray`)
* Topic (sub): `amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`)

## TODO / Limitations
* `nav2points.py` location resolution placeholder – needs adaptation to parameter structure or a resolver utility.
* Costmap clearing not yet ported (would use `/clear_entirely_local_costmap` etc. lifecycle-safe services in Nav2 if required).
* No QoS overrides set; if reliability is required for markers or latched behavior, adjust QoS profiles.

## Build
```bash
colcon build --packages-select alterego_navigation
source install/setup.bash
```

## License
BSD-3-Clause
