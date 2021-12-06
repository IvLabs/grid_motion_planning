#!/usr/bin/env python3
import rospy
import roslib
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf
import math
from math import sin, cos, pi,tan, atan2
import numpy as np
from pylab import *
from itertools import groupby
from operator import itemgetter
import matplotlib.pyplot as plt
from scipy import interpolate
from localmap import localmap
pose=[-2.0,-0.5,0.0]
#***********************************************************************
def handle_robot_pose(parent, child, pose):
    br = tf.TransformBroadcaster()
    br.sendTransform((pose[0], pose[1], 0), tf.transformations.quaternion_from_euler(0, 0, pose[2]), rospy.Time.now(), child,parent)

#***********************************************************************
def odometryCb(msg):
    global pose
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z
    theta=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    pose=[x,y,theta]

#*********************************************************************** 
def scanCb(msg):
    py,px=[],[]
    scandata=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    angle_increment=msg.angle_increment
    range_min=msg.range_min
    range_max=msg.range_max 
   
    m.updatemap(scandata,angle_min,angle_max,angle_increment,range_min,range_max,pose)
    handle_robot_pose("/map", "/mavros/odometry/in", pose)

#***********************************************************************    
def mappublisher(m,height, width, resolution,morigin):
    msg = OccupancyGrid()
    msg.header.frame_id='/map'
    msg.info.resolution = resolution
    msg.info.width      = math.ceil(width/resolution)
    msg.info.height     = math.ceil(height/resolution)
    msg.info.origin.position.x=-morigin[0]
    msg.info.origin.position.y=-morigin[1]
    msg.data = m
    mappub.publish(msg)

if __name__ == "__main__":

    rospy.init_node('main', anonymous=True) #make node 
    rospy.Subscriber('/mavros/odometry/in',Odometry,odometryCb)
    rospy.Subscriber("/laser/scan", LaserScan, scanCb)
    mappub= rospy.Publisher('/map',OccupancyGrid,queue_size=1)

    rate = rospy.Rate(10) # 100hz   

    height, width, resolution=6,6,0.05
    morigin=[height/2.0,0.0]
    m=localmap(height, width, resolution,morigin)

    while not rospy.is_shutdown():
        mappublisher(m.localmap,height, width, resolution,morigin)
        rate.sleep()