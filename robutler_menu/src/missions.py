#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess
import numpy as np
from visualization_msgs.msg import *

state_pub = rospy.Publisher('mission_state', String, queue_size=10)

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

global color_detected

def callback_color(data):
    global color_detected

    if data.data == 'color_detected':
        color_detected =  True
    else:
        color_detected = False
    print(color_detected)

def callback(data):
    global color_detected
    rospy.loginfo('mission recived: ' + data.data)

    if data.data == "find a purple sphere":

        state_pub.publish('start mission..')

        state_pub.publish('start color detection')
        bashCommand = 'rosrun robutler_detection detect_color.py'
        detect_ball_process = subprocess.Popen(bashCommand.split())

        # go in position then rotate
        state_pub.publish('go in the double_room')
        bashCommand = "rosrun robutler_navigation go_to_specific_point_on_map.py -xy -7 3.2"
        nav_process = subprocess.Popen(bashCommand.split())
        output, error = nav_process.communicate()

        state_pub.publish('check over the bad')
        [qx, qy, qz, qw] = get_quaternion_from_euler(0,0,45) # to watch over the bad
        bashCommand = "rosrun robutler_navigation go_to_specific_point_on_map.py -xy -7 3.2 -quaternion "+str(qx)+' '+str(qy)+' '+str(qz)+' '+str(qw)
        nav_process = subprocess.Popen(bashCommand.split())
        output, error = nav_process.communicate()

        
        if color_detected: # rospy.Subscriber("color_detection", String, callback=callback_color):
            state_pub.publish('I ve found a purple sphere!')
        else:
            state_pub.publish('check over the table')
            [qx, qy, qz, qw] = get_quaternion_from_euler(0,0,90) # to watch over the bad
            bashCommand = "rosrun robutler_navigation go_to_specific_point_on_map.py -xy -7 3.2 -quaternion "+str(qx)+' '+str(qy)+' '+str(qz)+' '+str(qw)
            nav_process = subprocess.Popen(bashCommand.split())
            output, error = nav_process.communicate()
            # nav_process.wait()

            if color_detected: # rospy.Subscriber("color_detection", String,callback = callback_color):
                state_pub.publish('I ve found a purple sphere!')
            else:
                state_pub.publish('purple sphere not found in the double room')

        detect_ball_process.kill()


    elif data.data == "check if the pc is in the office":
        rospy.loginfo('start mission..')
    elif data.data == "check if there is someone in the living room":
        rospy.loginfo('start mission..')

    
def listener():

    rospy.init_node('listener_mission', anonymous=True)

    rospy.Subscriber("mission", String, callback)
    rospy.Subscriber("color_detection", String, callback=callback_color)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()