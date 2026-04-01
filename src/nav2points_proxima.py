#!/usr/bin/env python3

# Stdlib
import os
import sys
import time
import math
import numpy as np
import json
import random
import inspect
import subprocess
from time import sleep

# Third-party
import requests
from colorama import Fore
import tf

# ROS
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid


class Proxima:
    def __init__(self):

        self.credentials = {}
        self.load_credentials()

        # Config backend
        self.backend_url = f"http://127.0.0.1:{self.credentials.get('BACKEND_API_PORT')}/"
        self.auth_header = ""
        self.headers = {"Authorization": self.auth_header}

        # Stato
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        self.robot_status = None
        self.path_length = None
        self.goal_reached_published = False
        self.mission_is_active = False
        self.mission_to_be_aborted = False
        self.target = None
        self.target_wp = None
        self.first_encounter = True
        self.waypoints = []
        self.initial_wp = ""
        self.robot_need_relocalize = False

        # ROS (iniziati dopo init_node)
        self.rate = None
        self.goal_reached_pub = None
        self.navigation_errors_pub = None
        self.map_pub = None

    # ---------------------------
    # Inizializzazione ROS
    # ---------------------------
    def init_ros(self):
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.Subscriber('target_location', String, self.target_location_callback)
        self.goal_reached_pub = rospy.Publisher('goal_reached', String, queue_size=10)
        self.navigation_errors_pub = rospy.Publisher('navigation_errors', String, queue_size=10)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_latch_callback)
        self.map_pub = rospy.Publisher(f"/{self.robot_name}/map", OccupancyGrid, queue_size=1, latch=True)
        self.waypoints_pub = rospy.Publisher('/waypoints', PoseArray, queue_size=1, latch=True)
        # TF listener, broadcaster and robot_pose subscriber
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/robot_pose', PoseStamped, self.get_map_odom_tf)
 

    def load_credentials(self):
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(script_dir, "..", "license", ".env")
        
        if not os.path.exists(filename):
            return
        with open(filename) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                key, value = line.split("=", 1)
                self.credentials[key] = value
                

    # ---------------------------
    # HTTP helpers
    # ---------------------------
    def login(self):
        resp = requests.post(self.backend_url + "login", json={"username": self.credentials.get("BACKEND_USERNAME"), "password": self.credentials.get("BACKEND_PASSWORD")})
        if resp.ok:
            self.auth_header = resp.content.decode("utf-8")
            self.headers = {"Authorization": self.auth_header}
            rospy.loginfo(f"Login Success, Auth code: {self.auth_header}")
            return True
        rospy.logerr(f"Login Failed: {resp.status_code}")
        return False

    def logout(self):
        if self.post_request("logout", {}).ok:
            self.auth_header = ""
            self.headers = {"Authorization": self.auth_header}
            rospy.loginfo("Logout Success")
        else:
            rospy.logwarn("Logout Failed")

    def get_request(self, endpoint):
        resp = requests.get(self.backend_url + endpoint, headers=self.headers)
        #self.print_response("GET", endpoint, resp)
        return resp

    def post_request(self, endpoint, data):
        resp = requests.post(self.backend_url + endpoint, json=data, headers=self.headers)
        self.print_response("POST", endpoint, resp, data)
        return resp

    def print_response(self, method, endpoint, response, data=None):
        msg = f"{method} at time [{time.time()}] with code [{response.status_code}] on {endpoint} "
        msg += (str(data) if data else "")
        msg += " : " + response.content.decode("utf-8")
        print(msg)
        
    def status_transition(self, new_status):
        while True:
            resp = self.post_request("system-status", {"status": new_status})
            if resp.ok:
                break
            rospy.logwarn(f"System status POST failed ({resp.status_code}); retrying...")
            sleep(0.5)

        while True:
            status = self.get_request("system-status").json()["content"]["status"]
            rospy.loginfo(f"Waiting for {new_status}... Current status: " + status)
            if status == new_status:
                break
            sleep(0.5)

    # ---------------------------
    # Setup logico (backend + mappa)
    # ---------------------------
    def setup(self):
        # 1) Login
        if not self.login():
            sys.exit(1)
        
        # 2) Transizioni di stato Idle->PreMapping (carica la mappa sulla Gui scelta dal dal launcher proxima_nav_launch)
        self.status_transition("Idle")
        self.status_transition("PreMapping")
        
        # 3) Caricamento mappa da parametro ROS launch file
        map_file = rospy.get_param('/map_file', '')
        self.initial_wp = rospy.get_param('/initial_waypoint', '')
        rospy.loginfo(f"Loading navigation map from file: {map_file}")
        self.load_navigation_map(map_file)

        # 4) Transizioni di stato (PreMapping)->Idle->PreNavigation 
        self.status_transition("Idle")
        self.status_transition("PreNavigation")
        
        # 3) Caricamento mappa da parametro ROS
        # map_file = rospy.get_param('/map_file', '')
        # self.initial_wp = rospy.get_param('/initial_waypoint', '')
        # rospy.loginfo(f"Loading navigation map from file: {map_file}")
        # self.load_navigation_map(map_file)

        # 5) Modalità navigazione e localizzazione iniziale
        self.post_request("navigation-mode", {"type": "auto"})
        sleep(0.5)
        if self.initial_wp:
            self.localize_robot(self.initial_wp)
            sleep(1)

    def load_navigation_map(self, map_file):
        try:
            with open(map_file, 'r', encoding='utf-8') as f:
                navigation_map_data = json.load(f)
        except FileNotFoundError:
            rospy.logerr(f"Navigation map JSON file not found: {map_file}")
            sys.exit(1)
        except json.JSONDecodeError:
            rospy.logerr(f"Invalid JSON format in navigation map file: {map_file}")
            sys.exit(1)

        self.waypoints = []
        for wp in navigation_map_data.get("waypoints", []):
            self.waypoints.append(wp)

        # Invia la mappa adattata
        resp = self.post_request("navigation-map", {
            "map": navigation_map_data.get("map"),
            "x": navigation_map_data.get("x", 0.0),
            "y": navigation_map_data.get("y", 0.0),
            "yaw": navigation_map_data.get("yaw", 0.0),
            "waypoints": navigation_map_data.get("waypoints", []),
            "map_info": navigation_map_data.get("map_info", {}),
            "areas": navigation_map_data.get("areas", [])
        })
        
        if resp.status_code == 200:
    
            rospy.loginfo("Navigation map loaded successfully.")
        else:
            rospy.logerr("Failed to upload Navigation map.")
            rospy.logerr(resp.status_code)
            
    # ------------------------------------------------------------------------
    #  RVIZ 
    # ------------------------------------------------------------------------
    
    # VISUALIZZARE WAIPONTS --------------------------------------------------      
        
        # Pubblico i waypoints su /waypoints come PoseArray.
        # Pubblico i waypoint rispetto al frame map. 
        # NB: Nel file .json della mappa i wp sono espressi in world, dal file recupero x,y,yaw (posizione del frame map rispetto al world) e porto i wp espressi in map.
        
        # posa di MAP rispetto a WORLD (dal file mapppa json)
        map_x = float(navigation_map_data.get("x", 0.0))
        map_y = float(navigation_map_data.get("y", 0.0))
        map_yaw = float(navigation_map_data.get("yaw", 0.0))
        
        # T_w_map (map rispetto world) e sua inversa T_map_w
        T_w_map = self._se2(map_x, map_y, map_yaw)
        T_map_w = np.linalg.inv(T_w_map)
        
        # Converte ogni waypoint da world a map
        wp_world = self.waypoints
        wp_map = []
        for wp in wp_world:
            T_w_wp = self._se2(float(wp["x"]), float(wp["y"]), float(wp["yaw"]))
            T_map_wp = T_map_w @ T_w_wp

            wp_map.append({
                **wp,
                "x": float(T_map_wp[0, 2]),
                "y": float(T_map_wp[1, 2]),
                "yaw": float(self._yaw_from_T(T_map_wp))
            })
    
        # Salva waypoints convertiti (ora in MAP)
        self.waypoints = wp_map
        
        # Pubblica in frame map
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"
        for wp in self.waypoints:
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
    # --------------------------------------------------------------------------
    # VISUALIZZARE FRAME MAP-->ODOM (TO FIX serve API)  

    # Non posso usare questa funzione perchè il robot lato nostro crea già il TF odom-->base_link, e se pubblico anche map->base_link su rviz vedo sparire e riapparire il frame perchè ha duediversi parent.    il robot pubblicherà direttamente map->odom, questa funzione potrà essere riabilitata per pubblicare direttamente map->odom.
    def map_pose(self): 
        """
        GET /map-pose
        Ritorna:
          - pose: x, y, yaw (map pose: base_link wrt map)
        In caso di errore ritorna (None, None)
        """
        try:
            resp = self.get_request("map-pose")
            if not resp.ok:
                rospy.logwarn(f"GET map-pose failed ({resp.status_code})")
                return None, None

            content = resp.json().get("content", {})
            pose = content.get("pose", {})

            if not pose:
                rospy.logwarn("map-pose response without pose")
                return None

            return {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "yaw": float(pose.get("yaw", 0.0)),
            }

        except Exception as e:
            rospy.logerr(f"Failed to get map-pose: {e}")
            return None, None

    def publish_map_tf(self):
        pose = self.map_pose()
        if not pose:
            return

        quat = quaternion_from_euler(0.0, 0.0, pose["yaw"])
        self.tf_broadcaster.sendTransform(
            (pose["x"], pose["y"], 0.0),   # translation
            quat,                          # rotation
            rospy.Time.now(),
            "base_link",                   # child
            "map"                          # parent
        )
        
    # ---------------------------
    # Navigazione helpers
    # ---------------------------
    def localize_robot(self, wp_name, radius=None):
        
        # Localization logic:
        # - If robot is within `radius` meters of waypoint -> localize by waypoint.
        # - Else -> localize by current pose (call localize-pose).
        
        # Find waypoint info (waypoints are stored in meters)
        wp = next((w for w in (self.waypoints or []) if w.get("name") == wp_name), None)
        if not wp:
            rospy.logerr(f"Waypoint '{wp_name}' not found in loaded waypoints")
            return

        # Current robot pose from robot_status (expected in meters)
        current_pose = (self.robot_status or {}).get("pose")
        if not current_pose:
            # no pose available -> try localize by waypoint as fallback
            rospy.logwarn("Current pose unavailable, using waypoint localisation")
            self.post_request("localize-waypoint", {"map": "navigation_map", "waypoint": wp_name})
            return

        # radius threshold (meters): explicit arg or ROS param or default 0.2 m
        thresh = float(radius) if radius is not None else 0.2

        dx = current_pose.get("x", 0.0) - float(wp.get("x", 0.0))
        dy = current_pose.get("y", 0.0) - float(wp.get("y", 0.0))
        dist = math.hypot(dx, dy)

        if dist <= thresh:
            rospy.loginfo(f"Within {dist:.2f} m <= {thresh:.2f} m of waypoint '{wp_name}' -> localize by waypoint")
            self.post_request("localize-waypoint", {"map": "navigation_map", "waypoint": wp_name})
        else:
            rospy.loginfo(f"{dist:.2f} m from waypoint '{wp_name}' (> {thresh:.2f}) -> localize by current pose")
            self.post_request("localize-pose", {
                "map": "navigation_map",
                "x": float(current_pose.get("x", 0.0)),
                "y": float(current_pose.get("y", 0.0)),
                "yaw": float(current_pose.get("yaw", 0.0))
            })

    def calculate_path_length(self, actual_pose, target_pose):
        dx = target_pose["x"] - actual_pose["x"]
        dy = target_pose["y"] - actual_pose["y"]
        return math.hypot(dx, dy)

    def send_mission(self, wp_name):
        
        # 1) Rilocalizza il robot
        if self.robot_need_relocalize == False:
            # If an external component already localized the robot, skip localization
            rospy.loginfo(f"External localization present -> skipping relocalize")
        else:
            # Else localize robot by either waypoint or current pose.
            self.localize_robot(self.initial_wp)
        

        # 2) Inizia la missione verso il waypoint
        rospy.loginfo(f"Sending robot to waypoint '{wp_name}'")
        resp = self.post_request("mission", {
            "action": "start-mission",
            "waypoints": [{"name": wp_name, "radius": 0.0}],
            "map": "navigation_map"
        })
        if resp.status_code == 200:
            self.target_wp = next((w for w in self.waypoints if w["name"] == wp_name), None)
        else:
            self.mission_to_be_aborted = True

    def check_abort_mission(self):
        if self.mission_to_be_aborted:
            rospy.loginfo("Aborting Mission.")
            
            if self.mission_is_active:
                self.stop_mission()

            # Pubblica goal_reached
            self.goal_reached_pub.publish("ABORTED")
            
            self.mission_to_be_aborted = False

    def stop_mission(self):
        if self.mission_is_active == True:
            resp = self.post_request("mission", {"action": "stop-mission"})
            if resp.status_code == 200:
                self.mission_to_be_aborted = True
                self.mission_is_active = False
                rospy.loginfo("Mission stopped successfully.")
            else:
                rospy.logerr("Failed to stop mission.")

    # ---------------------------
    # Status Update
    # ---------------------------
    def handle_status_errors(self):
        errors = (self.robot_status or {}).get("errors", [])
        if not errors:
            return
        for error in errors:
            rospy.logerr(f"Navigation error: {error}")
            self.navigation_errors_pub.publish(error)
            if error == "PATH_NOT_FOUND":
                self.mission_to_be_aborted = True
            elif error == "LASER_ERROR":
                self.mission_to_be_aborted = True
            elif error in "LOCALIZATION_JUMP":
                self.mission_to_be_aborted = True
            elif error in "LOCALIZATION_TIMEOUT":
                self.localize_robot(self.initial_wp)
            elif error == "ROBOT_OUT_OF_MAP":
                self.mission_to_be_aborted = True
            elif error == "ROBOT_STUCK":
                pass
            # Pubblica gli errori di navigazione quando si verificano
            self.navigation_errors_pub.publish(error)

    def update_mission_status(self):
        # Salva lo stato precedente
        if not hasattr(self, "_prev_status"):
            self._prev_status = self.robot_status.get("status", None)
            return

        current_status = self.robot_status.get("status", None)
        prev_status = self._prev_status

        if current_status == "RUN":
            if self.mission_is_active == False:
                # Notifica inzio nuova missione
                rospy.loginfo("Mission is now ACTIVE (RUN)")
            
            # Imposta sempre la missione attiva nello stato RUN
            self.mission_is_active = True

        # Transizione da RUN a IDLE
        elif prev_status == "RUN" and current_status == "IDLE":
            
            # Se la Missione era attiva, è stata completata
            if self.mission_is_active == True:
                rospy.loginfo("Target Reached.")
                # Pubblica goal_reached
                self.goal_reached_pub.publish("SUCCEEDED")

                # Aggiorna il waypoint di partenza per la prossima missione
                self.initial_wp = self.target


            # Aggiorno lo stato della missione
            self.mission_is_active = False

            # Ho bisogno di rilocalizzarmi
            self.robot_need_relocalize = True

            rospy.loginfo("Mission is now INACTIVE (IDLE)")

        # Gestione abort mission            
        self.check_abort_mission()
        
        self._prev_status = current_status

    # Stampa le informazioni di log
    def print_log(self, verbose=True):
        status = self.robot_status.get("status", None)
        pose = self.robot_status.get("pose", {})
        x = pose.get("x", None)
        y = pose.get("y", None)
        yaw = pose.get("yaw", None)

        if status == "RUN":
            msg = f"Reaching target '{self.target}'"
            if self.path_length is not None:
                msg += f", distance to cover: {self.path_length:.2f} m"
        else:
            msg = f"System {status}"

        if verbose:
            if x is not None and y is not None and yaw is not None:
                msg += f" | Pose: x={x:.2f} m, y={y:.2f} m, yaw={math.degrees(yaw):.1f}°"
            else:
                msg += " | Pose: unavailable"
        rospy.loginfo(msg)

    # ---------------------------
    # Callback ROS
    # ---------------------------
    
    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        # Callback per la ricezione di un messaggio su /initialpose
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        rospy.loginfo(f"Sending localization in x={position.x:.2f}, y={position.y:.2f}, yaw={math.degrees(yaw):.1f}°")
        self.post_request("localize-pose", {
            "map": "navigation_map",
            "x": position.x,
            "y": position.y,
            "yaw": yaw
        })

        self.robot_need_relocalize = False

    def map_latch_callback(self, msg):
        if not hasattr(self, "_prev_map"):
            self._prev_map = None
        
        try:
            for i in range(len(msg.data)):
                if self._prev_map is None or msg.data[i] != self._prev_map.data[i]:
                    self._prev_map = msg
                    self.map_pub.publish(msg)
                    rospy.loginfo("Republished /map as latched message")
                    break
        except Exception as e:
            rospy.logerr(f"Failed to republish /map: {e}")

    def target_location_callback(self, msg: String):
        self.target = msg.data
        rospy.loginfo(f"Received target waypoint: {self.target}")
        
        self.goal_reached_published = False
        self.path_length = None
        
        # Controlla se c'è già una missione attiva e la termina
        if self.mission_is_active:
            self.mission_to_be_aborted = True
            self.check_abort_mission()
            sleep(1)

        self.send_mission(self.target)

    def get_map_odom_tf(self, msg: PoseStamped):
        """
        Receive /robot_pose (PoseStamped msg) defined as the base_link position w.r.t. to map frame.
        Subtracts odom->base_link to it and broadcasts a TF with parent=msg.header.frame_id (or 'map')
        and child="odom"
        """
        try:
            import tf.transformations as tft
            
            # Get transforms as 4x4 matrices
            (odom_trans, odom_rot) = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
            
            # Build transformation matrices
            T_map_base = tft.compose_matrix(
                translate=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                angles=tft.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                                msg.pose.orientation.z, msg.pose.orientation.w])
            )
            
            T_odom_base = tft.compose_matrix(
                translate=odom_trans,
                angles=tft.euler_from_quaternion(odom_rot)
            )
            
            # Compute map->odom = map->base * inv(odom->base)
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
    # Loop principale
    # ---------------------------
    def main_loop(self):
        while not rospy.is_shutdown():
            # self.publish_map_tf()  # Pubblica continuamente la TF map->base_link (per ora, in futuro pubblicherà direttamente map-->odom)
            self.robot_status = self.get_request("robot-status").json().get("content", {})
            # Errori di stato
            self.handle_status_errors()

            # Aggiorna lo stato della missione
            self.update_mission_status()
 
            # Calcola la distanza dal target
            if self.target_wp is not None and self.robot_status.get("pose"):
                self.path_length = self.calculate_path_length(self.robot_status["pose"], self.target_wp)
               
            # Stampa le informazioni di log
            self.print_log()

            # Rate control
            self.rate.sleep()

    # ---------------------------
    # Shutdown
    # ---------------------------
    def shutdown(self):
        rospy.loginfo("Stop Received")
        try:
            if self.mission_is_active:
                self.mission_to_be_aborted = True
                self.check_abort_mission()
        except Exception:
            pass
        try:
            self.post_request("navigation-mode", {"type": "external"})
        except Exception:
            pass
        try:
            self.logout()
        except Exception:
            pass


def main():
    rospy.init_node('navigation', anonymous=False)
    node = Proxima()
    rospy.on_shutdown(node.shutdown)
    node.init_ros()

    try:
        node.setup()
    except KeyboardInterrupt:
        pass

    try:
        node.main_loop()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("Exception thrown")
        node.logout()
        sys.exit(0)


if __name__ == '__main__':
    main()