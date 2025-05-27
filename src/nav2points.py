#!/usr/bin/env python3

# license removed for brevity

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
from geometry_msgs.msg import Pose, Point, Quaternion

class GoForwardAvoid():
    def __init__(self):
        # Get robot name from environment variable
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        rospy.init_node('navigation', anonymous=False)

        # Inizializzazione e setup
        self.path_length = None
        self.global_plan_length = None  # Add this to track global plan length
        self.goal_reached_published = False  # Variabile di stato per tenere traccia se il messaggio è già stato pubblicato
        self.is_on_mostra1 = False  # Track if robot has ever visited Mostra1

        # what to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        self._point_list = rospy.get_param('navigation/Locations')
        self._sleep_timer = rospy.Rate(1.0)
        
        # Aggiungi un subscriber al topic 'target_location'
        rospy.Subscriber('target_location', String, self.target_location_callback)
        rospy.Subscriber("move_base/TebLocalPlannerROS/local_plan", Path, self.path_callback)
        
        # Add subscription to global plan topic
        global_plan_topic = f"/{self.robot_name}/move_base/TebLocalPlannerROS/global_plan"
        rospy.Subscriber(global_plan_topic, Path, self.global_plan_callback)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.rate = rospy.Rate(1)  # 1 Hz
        
        self.goal_reached_pub = rospy.Publisher('goal_reached', Bool, queue_size=10)
        # Inizializza la variabile target
        self.target = None
        self.first_encounter = True
        
        # Numero di volte da pubblicare True quando il goal è raggiunto
        self.publish_true_count = 50

    # Add a callback for global plan
    def global_plan_callback(self, msg):
        self.global_plan_length = self.calculate_path_length(msg)
        rospy.logdebug(f"Global plan length: {self.global_plan_length}")
    
    def path_callback(self, msg):
        self.path_length = self.calculate_path_length(msg)

    def calculate_path_length(self, path):
        length = 0.0
        for i in range(len(path.poses) - 1):
            pose1 = path.poses[i].pose
            pose2 = path.poses[i + 1].pose
            dx = pose1.position.x - pose2.position.x
            dy = pose1.position.y - pose2.position.y
            length += math.sqrt(dx**2 + dy**2)
        return length

    def publish_initial_pose_from_location(self, location_name):
        mostra_data = None
        for location in self._point_list:
            if location_name in location:
                mostra_data = location[location_name]
                break
                
        if mostra_data is None:
            rospy.logerr(f"{location_name} not found in point list")
            return False
            
        position = mostra_data['position']
        orientation = mostra_data['orientation']
        
        # Create PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        # Set position and orientation
        pose_msg.pose.pose = Pose(
            position=Point(position['x'], position['y'], position['z']),
            orientation=Quaternion(orientation['x'], orientation['y'], orientation['z'], orientation['w'])
        )
        
        # Define covariance matrix
        pose_msg.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]
        
        # Publish the message
        self.initial_pose_pub.publish(pose_msg)
        rospy.loginfo(f"Published initial pose from {location_name}")
        
        # Give time for the pose to be processed
        rospy.sleep(1.0)
        return True

    def target_location_callback(self, msg):
        # Imposta la variabile target con il messaggio ricevuto
        self.target = msg.data
        self.goal_reached_published = False  # 
        self.path_length = None

        rospy.loginfo(f"Target location updated to: {self.target}")

        # Check if target is DockStation and if we've visited Mostra1
        if self.target == "DockStation":
            if self.is_on_mostra1:
                rospy.loginfo("Target is DockStation and robot has visited Mostra1. Setting initial pose from Mostra1...")
                if not self.publish_initial_pose_from_location("Mostra1"):
                    rospy.logwarn("Failed to set initial pose from Mostra1, continuing with navigation anyway")

        # tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        # allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        
        # Create a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        for location in self._point_list:
            print(location)
            if self.target in location:
                data = location[self.target]  # Usa self.target per accedere al punto corretto
                position = data['position']
                orientation = data['orientation']
                print(f"Target Position: {position}")
                print(f"Target Orientation: {orientation}")
                
                # Setup the goal
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = position['x']
                goal.target_pose.pose.position.y = position['y']
                goal.target_pose.pose.position.z = position['z']
                goal.target_pose.pose.orientation.x = orientation['x']
                goal.target_pose.pose.orientation.y = orientation['y']
                goal.target_pose.pose.orientation.z = orientation['z']
                goal.target_pose.pose.orientation.w = orientation['w']

                # Try sending goal until succeeded
                max_retries = 5  # Maximum number of retries
                retry_count = 0
                
                while retry_count < max_retries and not rospy.is_shutdown():
                    # Clear costmaps before sending the goal
                    rospy.wait_for_service(f"/{self.robot_name}/move_base/clear_costmaps")
                    try:
                        clear_costmaps = rospy.ServiceProxy(f"/{self.robot_name}/move_base/clear_costmaps", Empty)
                        clear_costmaps()
                        rospy.loginfo("Costmaps cleared successfully.")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")

                    # Start moving
                    self.move_base.send_goal(goal)
                    last_clear_time = rospy.Time.now()
                    
                    # Monitor progress
                    while not rospy.is_shutdown():
                        state = self.move_base.get_state()
                        if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED]:
                            break
                            
                        # Clear costmaps every 5 seconds while moving
                        current_time = rospy.Time.now()
                        if (current_time - last_clear_time).to_sec() >= 5.0:
                            try:
                                clear_costmaps = rospy.ServiceProxy(f"/{self.robot_name}/move_base/clear_costmaps", Empty)
                                clear_costmaps()
                                rospy.loginfo("Costmaps cleared during navigation.")
                                last_clear_time = current_time
                            except rospy.ServiceException as e:
                                rospy.logerr(f"Service call failed during navigation: {e}")
                        
                        self.goal_reached_pub.publish(False)
                        self.rate.sleep()
                    
                    # Handle the result
                    if state == GoalStatus.SUCCEEDED:
                        # Goal was successful, verify with global plan
                        if self.global_plan_length is not None and self.global_plan_length < 1.0:
                            rospy.loginfo(f"Goal confirmed with global plan check. Remaining plan: {self.global_plan_length}m")
                            rospy.loginfo("Goal reached successfully. Publishing True message...")
                            
                            # Update Mostra1 visited flag if this target was Mostra1
                            if self.target == "Mostra1":
                                self.is_on_mostra1 = True
                                rospy.loginfo("Recorded successful visit to Mostra1")
                            elif self.target == "DockStation":
                                self.is_on_mostra1 = False
                                rospy.loginfo("Recorded successful visit to DockStation")
                                
                            # Publish success
                            publish_rate = rospy.Rate(10)
                            for _ in range(self.publish_true_count):
                                if rospy.is_shutdown():
                                    break
                                self.goal_reached_pub.publish(True)
                                publish_rate.sleep()
                            # Break out of retry loop
                            break
                        else:
                            # False positive
                            rospy.logwarn(f"Possible false positive! Remaining global plan: {self.global_plan_length}m")
                            rospy.loginfo("Goal verification failed. Retrying...")
                            retry_count += 1
                    else:
                        # Goal was aborted or rejected, retry
                        status_text = "ABORTED" if state == GoalStatus.ABORTED else "REJECTED"
                        rospy.logwarn(f"Goal {status_text}. Retrying... (Attempt {retry_count+1}/{max_retries})")
                        retry_count += 1
                        # Wait a bit before retrying
                        rospy.sleep(1.0)
                
                # Check if we exhausted all retries
                if retry_count >= max_retries:
                    rospy.logerr(f"Failed to reach goal after {max_retries} attempts.")
                    self.goal_reached_pub.publish(False)
                    
    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    try:
        point_mb = GoForwardAvoid()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")