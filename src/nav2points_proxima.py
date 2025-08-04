#!/usr/bin/env python3

# license removed for brevity
from time import sleep
import requests
import sys
import time
from colorama import Fore
import inspect
import random

import rospy
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import *
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

        print("\n>> switch to PreNavigation")
        self.post_request("system-status", {"status": "PreNavigation"})

        system_status = self.get_request("system-status").json()["content"]["status"]

        while system_status != "PreNavigation":
            sleep(1)
            system_status = self.get_request("system-status").json()["content"]["status"]

        print("\n>> localize robot in waypoint 0")
        self.post_request("localize-waypoint", {"map": "navigation_map","waypoint": "arrivo"})
        
        sleep(1)

        # Initialize the ROS node
        rospy.init_node('navigation', anonymous=False)

        # Inizializzazione e setup
        self.path_length = None
        self.goal_reached_published = False  # Variabile di stato per tenere traccia se il messaggio è già stato pubblicato

        # what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        #self._point_list = rospy.get_param('navigation/Locations')
        
        # Aggiungi un subscriber al topic 'target_location'
        rospy.Subscriber('target_location', String, self.target_location_callback)
        
        #Ved. API Proxima per lunghezza percorso
        #rospy.Subscriber("move_base/TebLocalPlannerROS/local_plan", Path, self.path_callback)
        
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
    


    #Funzioni relative a calcolo lunghezza percorso
    # def path_callback(self, msg):
    #     self.path_length = self.calculate_path_length(msg)

 
    # def calculate_path_length(self, path):
    #     length = 0.0
    #     for i in range(len(path.poses) - 1):
    #         pose1 = path.poses[i].pose
    #         pose2 = path.poses[i + 1].pose
    #         dx = pose1.position.x - pose2.position.x
    #         dy = pose1.position.y - pose2.position.y
    #         length += math.sqrt(dx**2 + dy**2)
    #     return length

    def target_location_callback(self, msg):
        # Imposta la variabile target con il messaggio ricevuto
        self.target = msg.data
        self.goal_reached_published = False  # 
        self.path_length = None


        rospy.loginfo(f"Target location updated to: {self.target}")
       
        print("\n>> send robot to waypoint: ",self.target)
        self.post_request("mission", {"action": "start-mission", "waypoints": [{"name": self.target, "radius": 0.0}],"map": "navigation_map"})
        
        while not rospy.is_shutdown():
            robot_status = self.get_request("robot-status").json()["content"]["status"]

            if robot_status in ["IDLE"]:
                rospy.loginfo("Goal reached successfully.")
                break

            #Per cominciare a parlare 1 m prima dell'arrivo
            # if self.path_length is not None:
            #     if self.path_length < 1.5 and not self.goal_reached_published:
            #         self.goal_reached_pub.publish("SUCCEEDED")
            #         self.goal_reached_published = True  # 
            #         rospy.loginfo("Goal is within 1 meter.")

            rospy.sleep(1)

        if robot_status == "IDLE":
            self.goal_reached_pub.publish("SUCCEEDED")
            rospy.loginfo("Goal reached successfully.")
        else:
            rospy.loginfo("Failed to reach the goal.")
            self.goal_reached_pub.publish("ABORTED")
                # Interrompi la riproduzione della bag file


    
    
    def shutdown(self):
        rospy.loginfo("Stop")
        self.logout()


if __name__ == '__main__':
    try:
        point_mb = Proxima()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
        point_mb.logout()
        sys.exit(0)