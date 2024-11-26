#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Point, Quaternion

def publish_pose():
    # Inizializza il nodo
    rospy.init_node('pose_publisher', anonymous=True)

    # Crea un publisher per il topic '/initialpose'
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # Attendi che il publisher sia pronto
    rospy.sleep(1)

    # Leggi i dati di "Ingresso" da point_list
    point_list = rospy.get_param('navigation/Locations')
    ingresso_data = None
    for location in point_list:
        if 'Ingresso' in location:
            ingresso_data = location['Ingresso']
            break

    for location_dict in _point_list:
        if self.new_location in location_dict:
            print(self.new_location)
            sub_locations = location_dict[self.new_location]
            for sub_location_dict in sub_locations:
                if self.new_sub_location in sub_location_dict:
                    print(self.new_sub_location)                
                    data = sub_location_dict[self.new_sub_location]
                    position = data['position']
                    orientation = data['orientation']
                    rospy.loginfo(f"Target Position: {position}")
                    rospy.loginfo(f"Target Orientation: {orientation}")

    if ingresso_data is None:
        rospy.logerr("Ingresso not found in point list")
        return

    position = ingresso_data['position']
    orientation = ingresso_data['orientation']

    # Crea il messaggio PoseWithCovarianceStamped
    pose_msg = PoseWithCovarianceStamped()

    # Imposta l'intestazione del messaggio
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    # Imposta la posizione e l'orientamento
    pose_msg.pose.pose = Pose(
        position=Point(position['x'], position['y'], position['z']),
        orientation=Quaternion(orientation['x'], orientation['y'], orientation['z'], orientation['w'])
    )

    # Definisci la matrice di covarianza (usa una matrice di identità come esempio)
    pose_msg.pose.covariance = [
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
    ]

    # Pubblica il messaggio
    pub.publish(pose_msg)
    rospy.loginfo("Published initial pose for Ingresso")

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass