#!/usr/bin/env python3
from px4_offboard import flightModes, Controller
import rospy
from geometry_msgs.msg import Twist
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import numpy as np
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():

    # initiate node
    rospy.init_node('px4_teleop', anonymous=True)

    # flight mode object
    modes = flightModes()

    # controller object
    control = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, control.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, control.posCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    # Make sure the drone is armed
    while not control.state.armed:
        modes.setArm()
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(control.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    
    # ROS main loop
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            control.sp.position.x = control.local_pos.x + 1
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z
            control.sp.yaw = control.curr_yaw

        elif key == 's':
            control.sp.position.x = control.local_pos.x - 1
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z
            control.sp.yaw = control.curr_yaw

        elif key == 'a':
            control.sp.position.x = control.local_pos.x
            control.sp.position.y = control.local_pos.y + 1
            control.sp.position.z = control.local_pos.z
            control.sp.yaw = control.curr_yaw
            
        elif key == 'd':
            control.sp.position.x = control.local_pos.x 
            control.sp.position.y = control.local_pos.y - 1
            control.sp.position.z = control.local_pos.z
            control.sp.yaw = control.curr_yaw          
            
        elif key == '8':
            control.sp.position.x = control.local_pos.x
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z + 1
            control.sp.yaw = control.curr_yaw           

        elif key == '5':
            control.sp.position.x = control.local_pos.x
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z - 1
            control.sp.yaw = control.curr_yaw
        elif key == '4':
            control.sp.position.x = control.local_pos.x
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z
            control.sp.yaw = control.curr_yaw + 0.3
        elif key == '6':
            control.sp.position.x = control.local_pos.x
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z
            control.sp.yaw = control.curr_yaw - 0.3    

        elif key == 'enter':
            control.sp.position.x = control.local_pos.x 
            control.sp.position.y = control.local_pos.y
            control.sp.position.z = control.local_pos.z                       


        sp_pub.publish(control.sp)
        rate.sleep()

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)   
    try:
        main()	
    except rospy.ROSInterruptException:
        pass



