#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan

class MoveBaseSeq():

    def __init__(self):
        rospy.init_node('move_base_sequence')
        self.g_add = 0
        self.goallist = [[0.81, 2.77], [-0.81, 6.63], [-3.06, 8.39], [-4.8, 6.5], [-4.65, 6.39], [-3.42, 1.18], [-1.49, 4.18], [-4.2, 4.02], [-3.0, 2.0], [-6.0, 3.7], [-6.0, 3.9], [-4.9, 5.9], [-4.3, 4.0], [-6.0, 3.8], [-4.2, 4.1], [-4.4, 4.6], [-4.2, -4.4], [-4.5, -4.6], [-4.6, 4.3], [-4.2, 0.7], [-3.1, -2.1], [-5.3, 0.78], [0.9, 3.5], [-4.6, 3.4], [-4.3, 6.0]]



        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        twist = Twist()
        twist.angular.z = 0
        pub.publish(twist)
        rospy.sleep(3)

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available, trying again!")
            rospy.loginfo("Waiting for move_base action server...")      #!
            wait1 = self.client.wait_for_server(rospy.Duration(10.0))
            if not wait1:
                rospy.logerr("Action server not available, shutting down!")
                rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Researching for Standard-zelle at Random locations ...")
        self.movebase_client()

    def active_cb(self):
        print("Location pose "+str(self.g_add)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass
        #To print current pose at each feedback:
        #print("Feedback for goal "+str(self.g_add)+": "+str(feedback))
        #print("Feedback for goal pose "+str(self.g_add)+" received")

    def done_cb(self, status, result):

    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:         #!
            print("\n#####################\n")
            print("Location pose received a cancel request after it started executing, completed execution! ")
            print(str(self.goallist[self.g_add]))

            print("\n#####################\n")
            rospy.sleep(1)
            rospy.signal_shutdown("Detection")
            #self.g_add += 1
            #self.movebase_SkipPreviousGoal()

        if status == 3:         #!
            print("\n#####################\n")
            print("Location pose reached")
            print(str(self.goallist[self.g_add]))
            print("\n#####################\n")
            rospy.sleep(2)
            self.g_add += 1
            self.movebase_SkipPreviousGoal()           #Even if we are not skipping here, function is same

        if status == 4:      #!
            print("\n#####################\n")
            print("Location pose was aborted by the Action Server; taking on next goal")
            print(str(self.goallist[self.g_add]))
            print("\n#####################\n")

            rospy.sleep(1)
            self.g_add += 1
            self.movebase_SkipPreviousGoal()
            #rospy.signal_shutdown("Goal pose " + str(self.g_add) + " aborted, shutting down!")
            return

        if status == 5:      #!
            print("\n#####################\n")
            print("Location pose has been rejected by the Action Server; taking on next goal")
            print(str(self.goallist[self.g_add]))
            print("\n#####################\n")
            rospy.sleep(1)
            self.g_add += 1
            self.movebase_SkipPreviousGoal()
            #rospy.signal_shutdown("Goal pose " + str(self.g_add) + " rejected, shutting down!")
            return

        if status == 8:      #!
            print("\n#####################\n")
            print("Location pose received a cancel request before it started executing, successfully cancelled!")
            print(str(self.goallist[self.g_add]))
            print("\n#####################\n")
            rospy.sleep(1)
            self.g_add += 1
            self.movebase_SkipPreviousGoal()

    def movebase_SkipPreviousGoal(self):
        print("Location pose " + str(self.g_add) + " in in process.")
        rospy.sleep(2)
        if self.g_add < len(self.goallist):
            next_goal = MoveBaseGoal()
            next_goal.target_pose.header.frame_id = "map"
            next_goal.target_pose.header.stamp = rospy.Time.now()
            self.goal_no = self.goallist[self.g_add]
            next_goal.target_pose.pose.position.x = self.goal_no[0]
            next_goal.target_pose.pose.position.y = self.goal_no[1]
            next_goal.target_pose.pose.orientation.w = 1
            print("\n#####################\n")
            print("Sending  new location pose")
            print(str(self.goallist[self.g_add]))
            print("\n#####################\n")
            rospy.sleep(1)
            self.client.send_goal(next_goal)
            self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
        else:
            rospy.loginfo("finished searching all locations!")
            rospy.signal_shutdown("finished searching all locations!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        self.goal_no = self.goallist[self.g_add]
        goal.target_pose.pose.position.x = self.goal_no[0]
        goal.target_pose.pose.position.y = self.goal_no[1]
        goal.target_pose.pose.orientation.w = 1
        print("\n#####################\n")
        print("Sending new location POSE")
        print(str(self.goallist[self.g_add]))
        print("\n#####################\n")
        rospy.sleep(1)
        #self.client.send_goal(goal)
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()



if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
