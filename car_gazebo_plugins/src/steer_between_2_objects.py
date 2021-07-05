#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Twist
import math

publisher = rospy.Publisher('/r0/cmd_vel', Twist)

def callback(msg):
    
    closest_points = [0,0,100]

    print(msg)

    for i in range(len(msg.poses)):
        for j in range(len(msg.poses)-i-1):
            k = i+j+1
            x = abs(msg.poses[i].position.x - msg.poses[k].position.x)
            y = abs(msg.poses[i].position.y - msg.poses[k].position.y)
            distance_between = math.sqrt(pow(x,2)+pow(y,2))
            print("Distance Between")
            print(i)
            print(k)
            print(distance_between)
            
            if distance_between < closest_points[2]:
                closest_points[0] = i
                closest_points[1] = k
                closest_points[2] = distance_between

    print("done with loop")
    print(closest_points[0])  
    print(closest_points[1])          
    print(msg.poses[closest_points[0]].position.y)
    print(msg.poses[closest_points[1]].position.y)
    target = abs(msg.poses[closest_points[0]].position.y) - abs(msg.poses[closest_points[1]].position.y)
    print(target)

    tw = Twist()

    tw.linear.x = 0
    if len(msg.poses) >= 2:
        tw.linear.x = 0.2

    tw.angular.z = 0
    if abs(msg.poses[closest_points[0]].position.y) < abs(msg.poses[closest_points[1]].position.y):
        tw.angular.z = 1
    if abs(msg.poses[closest_points[0]].position.y) > abs(msg.poses[closest_points[1]].position.y):
        tw.angular.z = -1
    
    tw.linear.y = 0
    tw.linear.z = 0
    tw.angular.x = 0
    tw.angular.y = 0

    publisher.publish(tw)
        

        



def steering_between_2_objects():

    rospy.init_node('steering_between_2_objects')
    sub = rospy.Subscriber('/clusters', PoseArray, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        steering_between_2_objects()
    except rospy.ROSInterruptException:
        pass
