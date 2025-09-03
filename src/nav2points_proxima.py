#!/usr/bin/env python3

# Stdlib
import os
import sys
import time
import math
import json
import random
import inspect
import subprocess
from time import sleep

# Third-party
import requests
from colorama import Fore

# ROS
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion


class Proxima:
    def __init__(self):
        # Config backend
        self.backend_url = "http://127.0.0.1:1984/"
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

        # ROS (iniziati dopo init_node)
        self.rate = None
        self.goal_reached_pub = None
        self.navigation_errors_pub = None

    # ---------------------------
    # Inizializzazione ROS
    # ---------------------------
    def init_ros(self):
        self.rate = rospy.Rate(1)  # 1 Hz
        rospy.Subscriber('target_location', String, self.target_location_callback)
        self.goal_reached_pub = rospy.Publisher('goal_reached', String, queue_size=10)
        self.navigation_errors_pub = rospy.Publisher('navigation_errors', String, queue_size=10)

    # ---------------------------
    # HTTP helpers
    # ---------------------------
    def login(self):
        resp = requests.post(self.backend_url + "login", json={"username": "admin", "password": "admin"})
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
        #self.print_response("POST", endpoint, resp, data)
        return resp

    def print_response(self, method, endpoint, response, data=None):
        msg = f"{method} at time [{time.time()}] with code [{response.status_code}] on {endpoint} "
        msg += (str(data) if data else "")
        msg += " : " + response.content.decode("utf-8")
        print(msg)

    # ---------------------------
    # Setup logico (backend + mappa)
    # ---------------------------
    def setup(self):
        # 1) Login
        if not self.login():
            sys.exit(1)

        # 2) Transizioni di stato
        self.post_request("system-status", {"status": "Idle"})
        sleep(1)
        self.post_request("system-status", {"status": "PreNavigation"})
        # attesa PreNavigation
        while True:
            status = self.get_request("system-status").json()["content"]["status"]
            if status == "PreNavigation":
                break
            sleep(0.5)

        # 3) Caricamento mappa da parametro ROS
        map_file = rospy.get_param('/map_file', '')
        self.initial_wp = rospy.get_param('/initial_waypoint', '')
        rospy.loginfo(f"Loading navigation map from file: {map_file}")
        self.load_navigation_map(map_file)

        # 4) Modalità navigazione e localizzazione iniziale
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

        # Adattamento coordinate in metri, yaw in radianti
        info = navigation_map_data.get("info", {})
        resolution = info.get("resolution", 1.0)
        height = info.get("size", {}).get("height", 0.0)

        self.waypoints = []
        for wp in navigation_map_data.get("waypoints", []):
            wp["x"] = wp.get("x", 0.0) * resolution
            # valore assoluto della differenza dall’origine in alto: |height - y|
            wp["y"] = abs(height - wp.get("y", 0.0)) * resolution
            wp["yaw"] = math.radians(-wp.get("yaw", 0.0))
            self.waypoints.append(wp)

        # Invia la mappa adattata
        self.post_request("navigation-map", {
            "map": navigation_map_data.get("map"),
            "waypoints": self.waypoints,
            "info": navigation_map_data.get("info", {})
        })

        rospy.loginfo("Navigation map loaded successfully.")

    # ---------------------------
    # Navigazione helpers
    # ---------------------------
    def localize_robot(self, wp_name):
        rospy.loginfo(f"Sending Localization to robot at waypoint '{wp_name}'")
        self.post_request("localize-waypoint", {"map": "navigation_map", "waypoint": wp_name})

    def calculate_path_length(self, actual_pose, target_pose):
        dx = target_pose["x"] - actual_pose["x"]
        dy = target_pose["y"] - actual_pose["y"]
        return math.hypot(dx, dy)

    def send_mission(self, wp_name):
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
            elif error in ("LOCALIZATION_JUMP", "LOCALIZATION_TIMEOUT"):
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