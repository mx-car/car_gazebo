#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import LaserScan
import math

ps = Pose()
publisher = rospy.Publisher('clusters', PoseArray)

def callback(msg):
    #print(msg.ranges)

    pinf = float('+inf')
    filteredMsg = []
    for i in range(len(msg.ranges)):
        #if msg.ranges[i] != pinf:
        filteredMsg.append(msg.ranges[i])

    clusterCount = 0
    clusterArray = []
    for i in range(len(filteredMsg)):
        if i > 0:
            dis = abs(filteredMsg[i-1] - filteredMsg[i])
            if dis > rospy.get_param('point_distance', 0.04):
                clusterCount = clusterCount + 1
        
        clusterArray.append(clusterCount)

    print(filteredMsg)
    print(clusterArray)

    start = 0
    clusterSize = 0
    clusterPos = []

    for i in range(len(clusterArray)):
        if i > 0:
            if clusterArray[i] == clusterArray[i-1]:
                clusterSize = clusterSize + 1
            else:
                clusterPos.append(((i-start)/2)+start)
                start = i


    print(clusterPos)

    # publisher = rospy.Publisher('poseArrayMessage', PoseArray)
    ps = PoseArray()
    ps.header.frame_id = msg.header.frame_id
    ps.header.stamp = msg.header.stamp
    
    print(msg)

    for i in range(len(clusterPos)):
        if filteredMsg[int(round(clusterPos[i],0))] < rospy.get_param('cluster_distance_cut_off', 2):
            pose = Pose()
            pose.position.x = math.sin(msg.angle_increment*clusterPos[i]) * filteredMsg[int(round(clusterPos[i],0))]               #clusterPos[i]/100
            pose.position.y = -math.cos(msg.angle_increment*clusterPos[i]) * filteredMsg[int(round(clusterPos[i],0))]               #math.cos(filteredMsg[int(round(clusterPos[i],0))] * msg.angle_increment)
            pose.position.z = 0
            pose.orientation.x = 0.365
            pose.orientation.y = 0
            pose.orientation.z = 0.4
            pose.orientation.w = 0
            ps.poses.append(pose)

    print(ps)
    publisher.publish(ps)
        

        



def laserToCluster():

    # publisher = rospy.Publisher('poseArrayMessage', PoseArray)
    rospy.init_node('laserToCluster')
    sub = rospy.Subscriber('/camera/scan', LaserScan, callback)
    rospy.spin()
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     publisher.publish(ps)
    #     rate.sleep()
    # # sub = rospy.Subscriber('/camera/scan', LaserScan, callback)
    # # rospy.spin()

if __name__ == '__main__':
    try:
        laserToCluster()
    except rospy.ROSInterruptException:
        pass
