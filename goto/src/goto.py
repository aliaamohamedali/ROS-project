#!/usr/bin/env python
import ssl
ssl._create_default_https_context = ssl._create_unverified_context


import rospy
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


rospy.init_node('goto', anonymous=True)

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

    def goto(self, pose):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

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
        
   
   
def go(pose):
    navigator = GoToPose()
    success = navigator.goto(pose)
    
    if success:
        rospy.loginfo("Benzo reached the desired pose")
    else:
        rospy.loginfo("Benzo failed to reach the desired pose")
        
 
def callback(position):
    go(position)
       
rospy.Subscriber("/goto_position", Pose, callback)


while not rospy.is_shutdown():
  rospy.spin()
