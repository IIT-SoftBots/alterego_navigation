#!/usr/bin/env python3

# Stdlib
import os
import sys
import time
import math
import numpy as np
import json
from time import sleep

# Third-party
import requests

# ROS
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# Todo: 
# 1) Failed to send custom waypoint mission: 500 (/custom-waypoint-mission) -> non va a buon fine se mando una missione custom, sembra un problema del backend che non accetta la richiesta, da investigare
# 2) Gestione errori: se ricevo errori di navigazione, oltre a pubblicarli su /navigation_errors, potrei voler abortire la missione (es. PATH_NOT_FOUND, LASER_ERROR, LOCALIZATION_JUMP, ROBOT_OUT_OF_MAP) o rilocalizzare (es. LOCALIZATION_TIMEOUT) a seconda del tipo di errore. Da implementare nella funzione handle_status_errors() che viene chiamata ad ogni aggiornamento di stato.  

class ProximaNavigationCore:
    """
    Core navigation logic: mission management, localization, error handling, backend communication.
    Visualization and RViz integration are delegated to the separate RvizBridge module.
    """
    
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
        self.target_type = None          # "waypoint" oppure "custom_pose"
        self.initial_custom_pose = None  # ultima posa custom raggiunta
        self.first_encounter = True
        self.waypoints = []
        self.initial_wp = ""
        self.robot_need_relocalize = False
        self.global_path_cached = False
        self.last_global_path_attempt_ts = 0.0
        self.global_path_retry_sec = 1.0
        self.active_navigation_errors = set()

        # ROS (iniziati dopo init_node)
        self.rate = None
        self.goal_reached_pub = None
        self.navigation_errors_pub = None
        self.global_path_world_pub = None

    # ---------------------------
    # Inizializzazione ROS
    # ---------------------------
    def init_ros(self):
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.Subscriber('target_location', String, self.target_location_callback)
        rospy.Subscriber('custom_target_world', PoseStamped, self.custom_target_world_callback)
        rospy.Subscriber('localize_pose_world', PoseStamped, self.localize_pose_world_callback)
        
        self.goal_reached_pub = rospy.Publisher('goal_reached', String, queue_size=10)
        self.navigation_errors_pub = rospy.Publisher('navigation_errors', String, queue_size=10)
        self.global_path_world_pub = rospy.Publisher('/mission_control/global_path_world', String, queue_size=1, latch=True)

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
        resp = requests.post(
            self.backend_url + "login",
            json={
                "username": self.credentials.get("BACKEND_USERNAME"),
                "password": self.credentials.get("BACKEND_PASSWORD")
            }
        )
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
        return resp

    def post_request(self, endpoint, data):
        resp = requests.post(self.backend_url + endpoint, json=data, headers=self.headers)
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
            rospy.loginfo(f"Waiting for {new_status}... Current status: {status}")
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
        
        # 2) Transizioni di stato Idle->PreMapping 
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

        rospy.loginfo(f"WAYPOINTS letti dal JSON: {self.waypoints}")
        rospy.loginfo(f"Numero waypoint: {len(self.waypoints)}")

        # Invia la mappa adattata al backend
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

        # radius threshold (meters)
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
            
    def relocalize_robot_before_mission(self):
        """
        Preserva la logica esistente:
        - se l'ultima posizione valida è un waypoint, usa localize_robot(initial_wp)
        - se l'ultima posizione valida è una posa custom, usa localize-pose
        """
        if self.robot_need_relocalize == False:
            rospy.loginfo("External localization present -> skipping relocalize")
            return

        if self.initial_custom_pose is not None:
            rospy.loginfo(
                "Relocalizing from last custom pose: "
                f"x={self.initial_custom_pose['x']:.2f}, "
                f"y={self.initial_custom_pose['y']:.2f}, "
                f"yaw={math.degrees(self.initial_custom_pose['yaw']):.1f}°"
            )
            self.post_request("localize-pose", {
                "map": "navigation_map",
                "x": float(self.initial_custom_pose["x"]),
                "y": float(self.initial_custom_pose["y"]),
                "yaw": float(self.initial_custom_pose["yaw"])
            })
            return

        if self.initial_wp:
            self.localize_robot(self.initial_wp)
            return

        rospy.logwarn("No initial waypoint or custom pose available for relocalization")

    def calculate_path_length(self, actual_pose, target_pose):
        dx = target_pose["x"] - actual_pose["x"]
        dy = target_pose["y"] - actual_pose["y"]
        return math.hypot(dx, dy)

    def send_mission(self, wp_name):
        self.post_request("navigation-mode", {"type": "auto"})
        sleep(0.5)

        # 1) Rilocalizza il robot
        self.relocalize_robot_before_mission()

        # 2) Inizia la missione verso il waypoint
        rospy.loginfo(f"Sending robot to waypoint '{wp_name}'")
        resp = self.post_request("mission", {
            "action": "start-mission",
            "waypoints": [{"name": wp_name, "radius": 0.0}],
            "map": "navigation_map"
        })

        if resp.status_code == 200:
            self.target_type = "waypoint"
            self.target = wp_name
            self.target_wp = next((w for w in self.waypoints if w["name"] == wp_name), None)
            self.global_path_cached = False
            self.last_global_path_attempt_ts = 0.0
            self.try_update_global_path(force=True)
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
            
    def send_custom_mission(self, x, y, yaw):
        self.post_request("navigation-mode", {"type": "auto"})
        sleep(0.5)

        # stessa logica pre-missione di send_mission
        self.relocalize_robot_before_mission()

        rospy.loginfo(
            f"Sending custom mission to x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°"
        )

        resp = self.post_request("custom-waypoint-mission", {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "map": "navigation_map"
        })

        if resp.status_code == 200:
            self.target_type = "custom_pose"
            self.target = "custom_pose"
            self.target_wp = {
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
                "name": "custom_pose"
            }
            self.global_path_cached = False
            self.last_global_path_attempt_ts = 0.0
            self.try_update_global_path(force=True)
            return True

        rospy.logerr(f"Failed to send custom waypoint mission: {resp.status_code}")
        self.mission_to_be_aborted = True
        return False

    def stop_mission(self):
        if self.mission_is_active == True:
            resp = self.post_request("mission", {"action": "stop-mission"})
            if resp.status_code == 200:
                self.mission_to_be_aborted = True
                self.mission_is_active = False
                rospy.loginfo("Mission stopped successfully.")
            else:
                rospy.logerr("Failed to stop mission.")

    def global_path(self):
        """
        GET /global-path
        Ritorna:
          - path: lista di punti [{"x": ..., "y": ...}, ...]
        In caso di errore ritorna None
        """
        try:
            resp = self.get_request("global-path")
            if not resp.ok:
                rospy.logwarn(f"GET global-path failed ({resp.status_code})")
                return None

            content = resp.json().get("content", {})
            path = content.get("path", None)

            if path is None:
                rospy.logwarn("global-path response without 'path'")
                return None

            return path

        except Exception as e:
            rospy.logerr(f"Failed to get global-path: {e}")
            return None

    def try_update_global_path(self, force=False):
        if self.target_wp is None:
            return

        if self.global_path_cached and not force:
            return

        now = time.time()
        if not force and (now - self.last_global_path_attempt_ts) < self.global_path_retry_sec:
            return

        self.last_global_path_attempt_ts = now
        path_points = self.global_path()
        if not path_points:
            return

        # Pubblica il path in world frame come JSON string
        self.global_path_world_pub.publish(json.dumps(path_points))
        self.global_path_cached = True
        rospy.loginfo(f"Global path cached with {len(path_points)} points")

    # ---------------------------
    # Status Update
    # ---------------------------
    def handle_status_errors(self):
        status = (self.robot_status or {}).get("status", None)
        if status != "ERROR":
            self.active_navigation_errors.clear()
            return

        errors = (self.robot_status or {}).get("errors", [])
        if not errors:
            self.active_navigation_errors.clear()
            return

        current_errors = set(errors)
        new_errors = []
        seen_in_cycle = set()
        for error in errors:
            if error in seen_in_cycle:
                continue
            seen_in_cycle.add(error)
            if error not in self.active_navigation_errors:
                new_errors.append(error)

        for error in new_errors:
            rospy.logerr(f"Navigation error: {error}")
            self.navigation_errors_pub.publish(error)

            if error == "PATH_NOT_FOUND":
                self.mission_to_be_aborted = True
            elif error == "LASER_ERROR":
                self.mission_to_be_aborted = True
            elif error == "LOCALIZATION_JUMP":
                self.mission_to_be_aborted = True
            elif error == "LOCALIZATION_TIMEOUT":
                self.relocalize_robot_before_mission()
            elif error == "ROBOT_OUT_OF_MAP":
                self.mission_to_be_aborted = True
            elif error == "ROBOT_STUCK":
                rospy.logwarn("ROBOT_STUCK detected: invalidating global path cache")
                self.global_path_cached = False
                self.last_global_path_attempt_ts = 0.0

        self.active_navigation_errors = current_errors
            
    def update_mission_status(self):
        # Salva lo stato precedente
        if not hasattr(self, "_prev_status"):
            self._prev_status = self.robot_status.get("status", None)
            return
    
        current_status = self.robot_status.get("status", None)
        prev_status = self._prev_status
    
        if prev_status == "ERROR" and current_status == "RUN":
            rospy.loginfo("Mission resumed (ERROR -> RUN)")
            self.mission_is_active = True

            # Refresh esplicito del path dopo eventuale replanning
            if self.target_wp is not None:
                self.global_path_cached = False
                self.last_global_path_attempt_ts = 0.0
                self.try_update_global_path(force=True)

        elif current_status == "RUN":
            if self.mission_is_active == False:
                # Notifica inzio nuova missione
                rospy.loginfo("Mission is now ACTIVE (RUN)")
            
            # Imposta sempre la missione attiva nello stato RUN
            self.mission_is_active = True

        # Transizione da RUN a ERROR
        elif prev_status == "RUN" and current_status == "ERROR":
            rospy.logwarn("Mission entered ERROR state (RUN -> ERROR)")
            # Missione ancora attiva: puo' riprendere con ERROR -> RUN
            self.mission_is_active = True
    
        # Transizione da RUN a IDLE
        elif prev_status == "RUN" and current_status == "IDLE":
            
            # Se la Missione era attiva, è stata completata
            if self.mission_is_active == True:
                rospy.loginfo("Target Reached.")
                # Pubblica goal_reached
                self.goal_reached_pub.publish("SUCCEEDED")
    
                # Caso classico: missione verso waypoint
                if self.target_type == "waypoint":
                    self.initial_wp = self.target
                    self.initial_custom_pose = None
    
                # Nuovo caso: missione verso posa custom
                elif self.target_type == "custom_pose" and self.target_wp is not None:
                    self.initial_custom_pose = {
                        "x": float(self.target_wp["x"]),
                        "y": float(self.target_wp["y"]),
                        "yaw": float(self.target_wp["yaw"])
                    }
    
                # Fallback conservativo: preserva la vecchia logica
                elif self.target is not None:
                    self.initial_wp = self.target

                # Fine missione: resetta il target attivo
                self.target_wp = None
                self.path_length = None
                self.target_type = None
    
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

        # rospy.loginfo(msg)

    # ---------------------------
    # Callback ROS (core only)
    # ---------------------------

    def target_location_callback(self, msg: String):
        self.target = msg.data
        rospy.loginfo(f"Received target waypoint: {self.target}")

        self.goal_reached_published = False
        self.path_length = None
        self.target_type = None
        # Controlla se c'è già una missione attiva e la termina
        if self.mission_is_active:
            self.mission_to_be_aborted = True
            self.check_abort_mission()
            sleep(1)

        self.send_mission(self.target)

    def custom_target_world_callback(self, msg: PoseStamped):
        """
        Callback per target custom ricevuto dal bridge RViz in world frame.
        """
        x_w = msg.pose.position.x
        y_w = msg.pose.position.y
        _, _, yaw_w = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        rospy.loginfo(
            f"Received custom target from bridge (world): "
            f"x={x_w:.2f}, y={y_w:.2f}, yaw={math.degrees(yaw_w):.1f}°"
        )

        self.goal_reached_published = False
        self.path_length = None
        self.target = None
        self.target_wp = None
        self.target_type = None

        # Controlla se c'è già una missione attiva e la termina
        if self.mission_is_active:
            self.mission_to_be_aborted = True
            self.check_abort_mission()
            sleep(1)

        self.send_custom_mission(x_w, y_w, yaw_w)

    def localize_pose_world_callback(self, msg: PoseStamped):
        """
        Callback per localizzazione ricevuta dal bridge RViz in world frame.
        """
        x_w = msg.pose.position.x
        y_w = msg.pose.position.y
        _, _, yaw_w = euler_from_quaternion([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        rospy.loginfo(
            f"Received localization from bridge (world): "
            f"x={x_w:.2f}, y={y_w:.2f}, yaw={math.degrees(yaw_w):.1f}°"
        )

        self.post_request("localize-pose", {
            "map": "navigation_map",
            "x": x_w,
            "y": y_w,
            "yaw": yaw_w
        })

        self.robot_need_relocalize = False

    # ---------------------------
    # Loop principale
    # ---------------------------
    def main_loop(self):
        while not rospy.is_shutdown():
            self.robot_status = self.get_request("robot-status").json().get("content", {})
            # Errori di stato
            self.handle_status_errors()

            # Aggiorna lo stato della missione
            self.update_mission_status()
 
            # Calcola la distanza dal target
            if self.target_wp is not None:
                self.try_update_global_path()

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
    rospy.init_node('navigation_core', anonymous=False)
    node = ProximaNavigationCore()
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
