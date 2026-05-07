#!/usr/bin/env python3

# Stdlib
import os
import sys
import math
import numpy as np
import json

# Third-party
import tf

# ROS
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


class RvizBridge:
    """
    RViz visualization and interaction bridge.
    Handles all visualization, TF broadcasting, and RViz user input.
    Communicates with the core navigation module via ROS topics.
    """
    
    def __init__(self):
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        
        # Map frame transformations
        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0
        self.T_w_map = np.eye(3, dtype=float)
        self.T_map_w = np.eye(3, dtype=float)
        
        # Waypoints visualization
        self.waypoints = []
        self.waypoints_map = []
        
        # Global path visualization
        self.global_path_cached = False
        self.last_global_path_attempt_ts = 0.0
        self.global_path_retry_sec = 1.0
        
        # ROS
        self.tf_listener = None
        self.tf_broadcaster = None
        self.rate = None
        
        # Publishers
        self.map_pub = None
        self.waypoints_pub = None
        self.waypoints_labels_pub = None
        self.global_path_pub = None
        self.custom_target_world_pub = None
        self.localize_pose_world_pub = None
        
        # Private state
        self._prev_map = None

    # ---------------------------
    # Inizializzazione ROS
    # ---------------------------
    def init_ros(self):
        self.rate = rospy.Rate(1)  # 1 Hz
        
        # TF listener, broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Subscribers (RViz inputs + core outputs)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.targetpose_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_latch_callback)
        rospy.Subscriber('/robot_pose', PoseStamped, self.get_map_odom_tf)
        rospy.Subscriber('/mission_control/global_path_world', String, self.global_path_world_callback)
        rospy.Subscriber('goal_reached', String, self.goal_reached_callback)
        
        # Publishers (outputs to core + visualization)
        self.map_pub = rospy.Publisher(f"/{self.robot_name}/map", OccupancyGrid, queue_size=1, latch=True)
        self.waypoints_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=1, latch=True)
        self.waypoints_labels_pub = rospy.Publisher('/waypoints_labels', MarkerArray, queue_size=1, latch=True)
        self.global_path_pub = rospy.Publisher('/mission_control/global_path', Marker, queue_size=1, latch=True)
        self.custom_target_world_pub = rospy.Publisher('custom_target_world', PoseStamped, queue_size=1)
        self.localize_pose_world_pub = rospy.Publisher('localize_pose_world', PoseStamped, queue_size=1)

    # ---------------------------
    # Frame transformations
    # ---------------------------
    
    def world_point_to_map(self, x_w, y_w, yaw_w=0.0):
        """
        Converte una posa da world a map usando T_map_w (inversa di T_w_map).
        Ritorna x, y, yaw nel frame map.
        """
        T_w_p = self._se2(float(x_w), float(y_w), float(yaw_w))
        T_map_p = self.T_map_w @ T_w_p
        return (
            float(T_map_p[0, 2]),
            float(T_map_p[1, 2]),
            float(self._yaw_from_T(T_map_p))
        )
    
    def map_point_to_world(self, x_m, y_m, yaw_m=0.0):
        """
        Converte una posa da map a world usando T_w_map.
        Ritorna x, y, yaw nel frame world.
        """
        T_map_p = self._se2(float(x_m), float(y_m), float(yaw_m))
        T_w_p = self.T_w_map @ T_map_p
        return (
            float(T_w_p[0, 2]),
            float(T_w_p[1, 2]),
            float(self._yaw_from_T(T_w_p))
        )
    
    def _se2(self, x, y, yaw):
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([
            [c, -s, x],
            [s,  c, y],
            [0,  0, 1]
        ], dtype=float)

    def _yaw_from_T(self, T):
        return math.atan2(T[1, 0], T[0, 0])
    
    # ---------------------------
    # Waypoints visualization
    # ---------------------------
    
    def publish_waypoints_in_map_frame(self):
        """
        Pubblica i waypoint nel frame map come PoseArray
        e i nomi come MarkerArray.
        I waypoint nel JSON sono in world, quindi qui li converto in map
        usando la funzione universale world_point_to_map().
        """

        # Converte ogni waypoint da world a map
        wp_world = self.waypoints
        wp_map = []

        for i, wp in enumerate(wp_world):
            x_m, y_m, yaw_m = self.world_point_to_map(
                wp["x"], wp["y"], wp["yaw"]
            )

            converted_wp = {
                **wp,
                "x": x_m,
                "y": y_m,
                "yaw": yaw_m
            }

            rospy.loginfo(
                f"Waypoint {i} -> name={converted_wp.get('name')}, "
                f"x={converted_wp.get('x'):.3f}, "
                f"y={converted_wp.get('y'):.3f}, "
                f"yaw={math.degrees(converted_wp.get('yaw')):.1f} deg"
            )

            wp_map.append(converted_wp)

        # Salva waypoints convertiti (ora in MAP)
        self.waypoints_map = wp_map

        # Pubblica frecce in frame map
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for wp in self.waypoints_map:
            pose = Pose()
            pose.position.x = wp["x"]
            pose.position.y = wp["y"]
            pose.position.z = 0.0

            quat = quaternion_from_euler(0.0, 0.0, wp["yaw"])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            pose_array.poses.append(pose)

        self.waypoints_pub.publish(pose_array)
        self.publish_waypoints_labels()

        rospy.loginfo("Published /waypoints and /waypoints_labels in frame 'map'")

    def publish_waypoints_labels(self):
        marker_array = MarkerArray()

        for i, wp in enumerate(self.waypoints_map):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoint_labels"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = wp["x"]
            marker.pose.position.y = wp["y"]
            marker.pose.position.z = 0.35  # testo un po' sopra la freccia

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.25  # altezza testo

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.text = wp.get("name", f"wp_{i}")

            marker_array.markers.append(marker)

        self.waypoints_labels_pub.publish(marker_array)

    # ---------------------------
    # Global path visualization
    # ---------------------------
    
    def publish_global_path(self, path_points):
        """
        Pubblica il global path come marker LINE_STRIP nel frame map.
        path_points è una lista di dict con chiavi x, y (in world frame).
        """
        if not path_points:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "global_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for i, point in enumerate(path_points):
            x_w = float(point.get("x", 0.0))
            y_w = float(point.get("y", 0.0))
            x_m, y_m, _ = self.world_point_to_map(x_w, y_w, 0.0)

            p = Pose()
            p.position.x = x_m
            p.position.y = y_m
            p.position.z = 0.01
            p.orientation.w = 1.0
            marker.points.append(p.position)

        self.global_path_pub.publish(marker)

    def try_update_global_path(self, path_points):
        """
        Pubblica il path ricevuto dal core (in world frame) 
        come marker visualization nel frame map.
        """
        if not path_points:
            return

        self.publish_global_path(path_points)
        self.global_path_cached = True
        rospy.loginfo(f"Global path published with {len(path_points)} points")

    def clear_global_path(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "global_path"
        marker.id = 0
        marker.action = Marker.DELETE
        self.global_path_pub.publish(marker)
        self.global_path_cached = False
        rospy.loginfo("Cleared /mission_control/global_path")
    
    # ---------------------------
    # Map republish (latched)
    # ---------------------------
    
    def map_latch_callback(self, msg):
        if not hasattr(self, "_prev_map"):
            self._prev_map = None

        try:
            if self._prev_map is None or len(msg.data) != len(self._prev_map.data):
                self._prev_map = msg
                self.map_pub.publish(msg)
                rospy.loginfo("Republished /map as latched message")
                return

            for i in range(len(msg.data)):
                if msg.data[i] != self._prev_map.data[i]:
                    self._prev_map = msg
                    self.map_pub.publish(msg)
                    rospy.loginfo("Republished /map as latched message")
                    break

        except Exception as e:
            rospy.logerr(f"Failed to republish /map: {e}")

    # ---------------------------
    # TF Bridge (map -> odom)
    # ---------------------------
    
    def get_map_odom_tf(self, msg: PoseStamped):
        """
        Receive /robot_pose (PoseStamped msg) defined as the base_link position w.r.t. map frame.
        Subtracts odom->base_link and broadcasts map->odom.
        """
        try:
            import tf.transformations as tft
            
            # Get transforms as 4x4 matrices
            (odom_trans, odom_rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            
            # Build transformation matrices
            T_map_base = tft.compose_matrix(
                translate=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                angles=tft.euler_from_quaternion([
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ])
            )

            T_odom_base = tft.compose_matrix(
                translate=odom_trans,
                angles=tft.euler_from_quaternion(odom_rot)
            )

            T_map_odom = np.dot(T_map_base, tft.inverse_matrix(T_odom_base))
            
            # Extract translation and rotation
            scale, shear, angles, trans, persp = tft.decompose_matrix(T_map_odom)
            quat = tft.quaternion_from_euler(*angles)

            self.tf_broadcaster.sendTransform(
                trans[:3].tolist(),
                quat.tolist(),
                msg.header.stamp,
                'odom',
                'map'
            )

        except Exception as e:
            rospy.logerr(f"get_map_odom_tf error: {e}")

    # ---------------------------
    # Callback ROS (RViz inputs)
    # ---------------------------

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """
        callback per la ricezione di un messaggio su /initialpose (2D Pose Estimate di RViz).
        Converte da map frame a world frame e pubblica sul topic custom_target_world.
        """
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw_map = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

        x_w, y_w, yaw_w = self.map_point_to_world(position.x, position.y, yaw_map)

        rospy.loginfo(
            f"RViz 2D Pose Estimate (map) -> converted to WORLD: "
            f"x={x_w:.2f}, y={y_w:.2f}, yaw={math.degrees(yaw_w):.1f}°"
        )

        # Pubblica sul topic per il core
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = x_w
        pose_msg.pose.position.y = y_w
        pose_msg.pose.position.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, yaw_w)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.localize_pose_world_pub.publish(pose_msg)

    def targetpose_callback(self, msg: PoseStamped):
        """
        Callback per la ricezione di un messaggio su /move_base_simple/goal
        RViz pubblica la posa nel frame map.
        Converte da map frame a world frame e pubblica sul topic custom_target_world.
        """
        position = msg.pose.position
        orientation = msg.pose.orientation
        _, _, yaw_map = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])

        rospy.loginfo(
            f"Received target pose from RViz in map frame: "
            f"x={position.x:.2f}, y={position.y:.2f}, yaw={math.degrees(yaw_map):.1f}°"
        )

        # Converte la posa da map a world prima di mandarla al backend
        x_w, y_w, yaw_w = self.map_point_to_world(position.x, position.y, yaw_map)

        rospy.loginfo(
            f"Converted RViz goal map->world: "
            f"x={x_w:.2f}, y={y_w:.2f}, yaw={math.degrees(yaw_w):.1f}°"
        )

        # Pubblica sul topic per il core
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = x_w
        pose_msg.pose.position.y = y_w
        pose_msg.pose.position.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, yaw_w)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.custom_target_world_pub.publish(pose_msg)

    def global_path_world_callback(self, msg: String):
        """
        Callback per il global path ricevuto dal core in world frame (JSON string).
        Pubblica il path come marker visualization nel frame map.
        """
        try:
            path_points = json.loads(msg.data)
            self.try_update_global_path(path_points)
        except Exception as e:
            rospy.logerr(f"Failed to parse global_path_world: {e}")

    def goal_reached_callback(self, msg: String):
        if msg.data in ("SUCCEEDED", "ABORTED"):
            self.clear_global_path()

    # ---------------------------
    # Setup visualizzazione (call solo una volta per caricare waypoints)
    # ---------------------------
    
    def setup_visualization(self, navigation_map_data):
        """
        Carica i dati della mappa e pubblica i waypoint.
        Deve essere chiamato UNA VOLTA dopo che il core ha caricato la mappa.
        """
        # posa di MAP rispetto a WORLD
        self.map_x = float(navigation_map_data.get("x", 0.0))
        self.map_y = float(navigation_map_data.get("y", 0.0))
        self.map_yaw = float(navigation_map_data.get("yaw", 0.0))

        # T_w_map (map rispetto world) e sua inversa T_map_w
        self.T_w_map = self._se2(self.map_x, self.map_y, self.map_yaw)
        self.T_map_w = np.linalg.inv(self.T_w_map)

        self.waypoints = []
        for wp in navigation_map_data.get("waypoints", []):
            self.waypoints.append(wp)

        rospy.loginfo(f"RvizBridge: Loaded {len(self.waypoints)} waypoints")
        
        # Pubblica i waypoint subito
        self.publish_waypoints_in_map_frame()

    # ---------------------------
    # Main loop (minimo, per periodic tasks se necessario)
    # ---------------------------
    
    def main_loop(self):
        """
        Loop minimo del bridge: principalmente passivo (callback-driven).
        Potrebbe essere usato per periodic refresh se necessario.
        """
        while not rospy.is_shutdown():
            # Per ora principalmente passive (callback driven)
            self.rate.sleep()

    # ---------------------------
    # Shutdown
    # ---------------------------
    def shutdown(self):
        rospy.loginfo("RvizBridge Stop Received")
        try:
            self.clear_global_path()
        except Exception:
            pass


def main():
    rospy.init_node('navigation_rviz_bridge', anonymous=False)
    bridge = RvizBridge()
    rospy.on_shutdown(bridge.shutdown)
    bridge.init_ros()

    # Attendi che il core carichi la mappa (opzionale: potrebbe aspettare un topic)
    # Per ora, carica la mappa localmente se disponibile
    map_file = rospy.get_param('/map_file', '')
    if map_file:
        try:
            with open(map_file, 'r', encoding='utf-8') as f:
                navigation_map_data = json.load(f)
                bridge.setup_visualization(navigation_map_data)
        except Exception as e:
            rospy.logwarn(f"RvizBridge: Could not load map_file: {e}")

    try:
        bridge.main_loop()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("RvizBridge Exception thrown")
        sys.exit(0)


if __name__ == '__main__':
    main()
