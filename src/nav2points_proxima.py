#!/usr/bin/env python3

# license removed for brevity
from time import sleep
import requests
import sys
import time
from colorama import Fore
import inspect
import random
import json

import rospy
import os
from std_srvs.srv import Empty
from std_msgs.msg import String  # Importa il tipo di messaggio String
from std_msgs.msg import Bool  # Importa il tipo di messaggio String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.msg import Path
import subprocess

class Proxima():
    def __init__(self):
        
        self.backend_url = "http://127.0.0.1:1984/"
        self.auth_header = ""
        self.headers = {"Authorization": self.auth_header}

        # Get robot name from environment variable
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')

        # Login to the backend
        if not self.login():
            sys.exit(1)

        print("\n>> switch to Idle")
        self.post_request("system-status", {"status": "Idle"})

        sleep(1)

        print("\n>> switch to PreNavigation")
        self.post_request("system-status", {"status": "PreNavigation"})

        system_status = self.get_request("system-status").json()["content"]["status"]

        while system_status != "PreNavigation":
            sleep(1)
            system_status = self.get_request("system-status").json()["content"]["status"]

        map_file = rospy.get_param('/map_file', '')
        print(f"Loading navigation map from: {map_file}")
        try:
            with open(map_file, 'r', encoding='utf-8') as json_file:
                navigation_map_data = json.load(json_file)
            
            # Get resolution value from info section
            resolution = navigation_map_data.get("info", {}).get("resolution", 1.0)
            
            # Multiply x and y coordinates by resolution
            for waypoint in navigation_map_data.get("waypoints", []):
                waypoint["x"] = waypoint["x"] * resolution
                waypoint["y"] = waypoint["y"] * resolution
            
### TODO ADJUST WAYPOINTS UNITS TO METERS, RADIANS FROM PIXELS            

            self.post_request("navigation-map", navigation_map_data)
        except FileNotFoundError:
            rospy.logerr(f"Navigation map JSON file not found: {map_file}")
            sys.exit(1)
        except json.JSONDecodeError:
            rospy.logerr(f"Invalid JSON format in navigation map file: {map_file}")
            sys.exit(1)

        sleep(1)

        initial_wp = rospy.get_param('/initial_waypoint', '')
        print(f"\n>> localize robot in waypoint {initial_wp}")
        self.post_request("localize-waypoint", {"map": "navigation_map","waypoint": initial_wp})
        
        sleep(10)

        # Initialize the ROS node
        rospy.init_node('navigation', anonymous=False)
        self.rate = rospy.Rate(1)  # 1 Hz

        # Inizializzazione e setup
        self.robot_status = None
        self.path_length = None
        self.goal_reached_published = False  # Variabile di stato per tenere traccia se il messaggio è già stato pubblicato
        self.mission_to_be_aborted = False

        # what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        #self._point_list = rospy.get_param('navigation/Locations')
        
        # Aggiungi un subscriber al topic 'target_location'
        rospy.Subscriber('target_location', String, self.target_location_callback)
        
        self.goal_reached_pub = rospy.Publisher('goal_reached', String, queue_size=10)
        # Inizializza la variabile target
        self.target = None
        self.first_encounter = True

    def login(self):
        response = requests.post(
            self.backend_url + "login", 
            json={"username": "admin", "password": "admin"},
        )
        if response.ok:
            self.auth_header = response.content.decode("utf-8")
            self.headers = {"Authorization": self.auth_header}
            print("Auth Header: ", self.auth_header)
            print("Login Success")
            return True
        else:
            print("Status Code: ", response.status_code)
            print("Response Content: ", response.content.decode("utf-8"))
            print("Login Failed")
            return False

    def logout(self):
        result = self.post_request("logout", {})
        if result:
            self.auth_header = ""
            self.headers = {"Authorization": self.auth_header}
            print("Logout Success")
        else:
            print("Logout Failed")

    def get_request(self, endpoint): 
        response = requests.get(self.backend_url + endpoint, headers=self.headers)
        self.print_response("GET", endpoint, response)
        return response

    def post_request(self, endpoint, data):
        response = requests.post(self.backend_url + endpoint, json=data, headers=self.headers)
        self.print_response("POST", endpoint, response, data)
        return response

    def print_response(self, request, endpoint, response, data=None):
        print(
            request + " at time [" + str(time.time()) 
            + "] with code [" 
            + str(response.status_code) + "] on " + endpoint + " "
            + (str(data) if data else "") + " : " + response.content.decode("utf-8")
        )
 
    def calculate_path_length(self):
        length = 0.0
        path_poses = self.get_request("global-path").json()["content"]["path"]
        rospy.loginfo(f"Path poses: {path_poses}")
        
        for i in range(len(path_poses) - 1):
            pose1 = path_poses[i].pose
            pose2 = path_poses[i + 1].pose
            dx = pose1.position.x - pose2.position.x
            dy = pose1.position.y - pose2.position.y
            length += math.sqrt(dx**2 + dy**2)
        return length

    def target_location_callback(self, msg):
        # Imposta la variabile target con il messaggio ricevuto
        self.target = msg.data
        self.goal_reached_published = False  # 
        self.path_length = None

        rospy.loginfo(f"Target location updated to: {self.target}")

        print("\n>> send robot to waypoint: ",self.target)
        response = self.post_request("mission", {"action": "start-mission", "waypoints": [{"name": self.target, "radius": 0.0}],"map": "navigation_map"})
        if response.status_code == 200:
            self.sent_waypoint = True  # Imposta la variabile per indicare che è stato inviato un waypoint
        else:
            self.mission_to_be_aborted = True  # Imposta la variabile per indicare che la missione deve essere abortita
        

    def main_loop(self):
        while not rospy.is_shutdown():
            self.robot_status = self.get_request("robot-status").json()["content"]

            if self.robot_status["errors"]:
                # Gestisco gli errori
                rospy.logerr(f"Errors during navigation: {self.robot_status['errors']}")

                if self.robot_status["errors"] in ["ROBOT_STUCK"]:
                    rospy.logerr("Robot is stuck, aborting mission.")
                    self.mission_to_be_aborted = True
                    


            # Gestione dello status: IDLE, RUN.. e pubblicazione goal_reached
            if self.mission_to_be_aborted == True:
                rospy.loginfo("Mission aborted due to previous errors.")
                self.goal_reached_pub.publish("ABORTED")
                self.mission_to_be_aborted = False

            # if self.has_to_navigate:

            #     # rospy.loginfo(f"Robot status: {self.robot_status}")
            #     if self.robot_status["errors"]:
            #         has_to_navigate = False  # Esci dal ciclo se ci sono errori   
                    
            #     if self.robot_status["status"] in ["IDLE"]:
            #         rospy.loginfo("Goal reached.")
            #         has_to_navigate = False

            #     #Per cominciare a parlare 1 m prima dell'arrivo
            #     self.path_length = self.calculate_path_length()
            #     if self.path_length is not None:
            #         if self.path_length < 1.5 and not self.goal_reached_published:
            #             self.goal_reached_pub.publish("SUCCEEDED")
            #             self.goal_reached_published = True  # 
            #             rospy.loginfo("Goal is within 1 meter.")
            
            # else:
        
            #     if self.robot_status["errors"]:
            #         rospy.logerr(f"Errors during navigation: {self.robot_status['errors']}")
            #     else:
            #         if self.robot_status["status"] == "IDLE":
            #             self.goal_reached_pub.publish("SUCCEEDED")
            #             rospy.loginfo("Goal reached successfully.")
            #         else:
            #             rospy.loginfo("Failed to reach the goal.")
            #             self.goal_reached_pub.publish("ABORTED")
            #                 # Interrompi la riproduzione della bag file

            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stop")
        self.logout()


if __name__ == '__main__':
    try:
        point_mb = Proxima()
        point_mb.main_loop()
        #rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
        point_mb.logout()
        sys.exit(0)