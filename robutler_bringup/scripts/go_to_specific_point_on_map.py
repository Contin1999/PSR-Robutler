#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import argparse

class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
        
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-xy',nargs='+', required=False,type=float, help='x and y coordinates of the desired destination')
    parser.add_argument('-quaternion',nargs='+', type=float , required=False, default=[0,0,0,1],help='orientation of the robot at the desired position in quaternion form: r1 r2 r3 r4')
    parser.add_argument('-room', required=False,type=str, help='name of the desired destination, possible rooms: kitchen\nliving_room\nsmall_bathroom\nbathroom\ndouble_room\nsingle_room\nstudy_room\nhallway')

    args = vars(parser.parse_args())

    apartment_rooms = { 'kitchen':[-3.5,-0.7,0],
                        'living_room': [-3.5,-4,0],
                        'small_bathroom': [1.5,-1.5,0],
                        'bathroom': [0.5,1,0],
                        'double_room': [-5,3,0],
                        'single_room': [-3,3,0],
                        'study_room': [-0.5,3,0],
                        'hallway': [-3,1.5,0] 
                        }
    
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        # position = {'x': -2.00, 'y' : -4.00}
        # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        if args['xy'] is not None:
            position = {'x':args['xy'][0] ,'y': args['xy'][1]}
            quaternion = {'r1' : args['quaternion'][0], 'r2' : args['quaternion'][1], 'r3' : args['quaternion'][2], 'r4' : args['quaternion'][3]}
        elif args['room'] is not None and args['room'] in apartment_rooms:
            x = apartment_rooms[args['room']][0]
            y = apartment_rooms[args['room']][1]
            position = {'x':x,'y': y}
            quaternion = {'r1' : args['quaternion'][0], 'r2' : args['quaternion'][1], 'r3' : args['quaternion'][2], 'r4' : args['quaternion'][3]}
        else:
            rospy.loginfo("no position required. Quitting")
            exit

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Reached the desired pose")
        else:
            rospy.loginfo("Failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")