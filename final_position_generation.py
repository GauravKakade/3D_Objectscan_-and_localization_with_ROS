#! /usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import actionlib
import math
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import geometry_msgs.msg


global side

def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "zelle_front_center"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.5
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.z = 1.0
    goal.target_pose.pose.orientation.w = 0  # was w = 0.0007963

    client.send_goal(goal)
    print("Goal sent")
    #print ("Destination at:", "x =", x3, "y =", y3)
    rospy.sleep(4)
    print("Execution finished")
    rospy.signal_shutdown("Detection")

def callback(msg):# for hypotenuse and theta
    #print msg
    global hypotenuse
    global theta
    hypotenuse = msg.x
    theta = msg.y
    #print hypotenuse


clientactionlib = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#clientactionlib.cancel_all_goals()


def callback3(data):
    #global x2
    #global y2
    global q_current
    x2 = round(float(data.pose.pose.position.x), 2)
    y2 = round(float(data.pose.pose.position.y), 2)
    q_current = data.pose.pose.orientation
    print('x2 =', x2, 'y2 =', y2)

def callback2(side_detected):  # for side name
    clientactionlib.cancel_all_goals()
    side = str(side_detected.data)
    #print side
    print("sleeping for 5 sec")
    rospy.sleep(5)

    if side == "left":
        if theta >= 0:
            print("Left Case 1: TurtleBot is to the left of left")
            theta_rad = math.radians(theta)
            y11 = (round(math.sin(theta_rad) * hypotenuse, 2)) * -1
            x11 = round(math.cos(theta_rad) * hypotenuse, 2)
            x1 = x11 + 0.35
            y1 = y11 - 0.4
            q_current.z = -1 * (q_current.z)
            print(q_current)

            q1 = [None] * 4
            q1[0] = q_current.x
            q1[1] = q_current.y
            q1[2] = q_current.z
            q1[3] = q_current.w

            q2 = [None] * 4
            q2[0] = 0
            q2[1] = 0
            q2[2] = -0.317392992493
            q2[3] = 0.94829409379

            qr = tf.transformations.quaternion_multiply(q1, q2)

            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "zelle_front_center"
            static_transformStamped.transform.translation.x = x1
            static_transformStamped.transform.translation.y = y1
            static_transformStamped.transform.translation.z = 0

            static_transformStamped.transform.rotation.x = 0
            static_transformStamped.transform.rotation.y = 0
            static_transformStamped.transform.rotation.z = qr[2]
            static_transformStamped.transform.rotation.w = qr[3]
            broadcaster.sendTransform(static_transformStamped)

            rospy.sleep(4)
            print("zelle_front_center frame created")
            movebase_client()

        if theta <= 0:
            print("Left Case 2: TurtleBot is to the right of Left")
            theta_rad = math.radians(abs(theta))
            y11 = round(math.sin(theta_rad) * hypotenuse, 1)
            x11 = round(math.cos(theta_rad) * hypotenuse, 1)
            x1 = x11 + 0.35
            y1 = y11 - 0.4

            print("x1=", x1, "y1=", y1)

            q_current.z = -1 * (q_current.z)
            print(q_current)

            q1 = [None] * 4
            q1[0] = q_current.x
            q1[1] = q_current.y
            q1[2] = q_current.z
            q1[3] = q_current.w

            q2 = [None] * 4
            q2[0] = 0
            q2[1] = 0
            q2[2] = -0.317392992493
            q2[3] = 0.94829409379

            qr = tf.transformations.quaternion_multiply(q1, q2)

            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "zelle_front_center"
            static_transformStamped.transform.translation.x = x1
            static_transformStamped.transform.translation.y = y1
            static_transformStamped.transform.translation.z = 0

            static_transformStamped.transform.rotation.x = 0
            static_transformStamped.transform.rotation.y = 0
            static_transformStamped.transform.rotation.z = qr[2]
            static_transformStamped.transform.rotation.w = qr[3]
            broadcaster.sendTransform(static_transformStamped)

            rospy.sleep(4)
            print("zelle_front_center frame created")
            movebase_client()

    if side == "front":
        if theta >= 0:
            print("Front Case 1: TurtleBot is to the left of Front")
            theta_rad = math.radians(theta)
            y1 = (round(math.sin(theta_rad) * hypotenuse, 2)) * -1
            x1 = round(math.cos(theta_rad) * hypotenuse, 2)

            q_current.z = -1 * (q_current.z)
            print(q_current)

            q1 = [None] * 4
            q1[0] = q_current.x
            q1[1] = q_current.y
            q1[2] = q_current.z
            q1[3] = q_current.w

            q2 = [None] * 4
            q2[0] = 0
            q2[1] = 0
            q2[2] = -0.317392992493
            q2[3] = 0.94829409379

            qr = tf.transformations.quaternion_multiply(q1, q2)


            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "zelle_front_center"
            static_transformStamped.transform.translation.x = x1
            static_transformStamped.transform.translation.y = y1
            static_transformStamped.transform.translation.z = 0

            static_transformStamped.transform.rotation.x = 0
            static_transformStamped.transform.rotation.y = 0
            static_transformStamped.transform.rotation.z = qr[2]
            static_transformStamped.transform.rotation.w = qr[3]
            broadcaster.sendTransform(static_transformStamped)

            rospy.sleep(4)
            print("zelle_front_center frame created")
            movebase_client()



        if theta <= 0:
            print("Front Case 2: TurtleBot is to the right of Front")
            theta_rad = math.radians(abs(theta))
            y1 = round(math.sin(theta_rad) * hypotenuse, 1)
            x1 = round(math.cos(theta_rad) * hypotenuse, 1)
            print("x1=", x1, "y1=", y1)

            q_current.z = -1 * (q_current.z)
            print(q_current)

            q1 = [None] * 4
            q1[0] = q_current.x
            q1[1] = q_current.y
            q1[2] = q_current.z
            q1[3] = q_current.w

            q2 = [None] * 4
            q2[0] = 0
            q2[1] = 0
            q2[2] = -0.317392992493
            q2[3] = 0.94829409379

            qr = tf.transformations.quaternion_multiply(q1, q2)

            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "zelle_front_center"
            static_transformStamped.transform.translation.x = x1
            static_transformStamped.transform.translation.y = y1
            static_transformStamped.transform.translation.z = 0

            static_transformStamped.transform.rotation.x = 0
            static_transformStamped.transform.rotation.y = 0
            static_transformStamped.transform.rotation.z = qr[2]
            static_transformStamped.transform.rotation.w = qr[3]
            broadcaster.sendTransform(static_transformStamped)

            rospy.sleep(4)
            print("zelle_front_center frame created")
            movebase_client()



    if side == "right":

        if theta >= 0:
            print("Right Case 1: TurtleBot is left of Right")
            theta_rad = math.radians(theta)
            y11 = (round(math.sin(theta_rad) * hypotenuse, 2)) * -1
            x11 = round(math.cos(theta_rad) * hypotenuse, 2)
            x1 = x11 + 0.35
            y1 = y11 + 0.4
            q_current.z = -1 * (q_current.z)
            print(q_current)

            q1 = [None] * 4
            q1[0] = q_current.x
            q1[1] = q_current.y
            q1[2] = q_current.z
            q1[3] = q_current.w

            q2 = [None] * 4
            q2[0] = 0
            q2[1] = 0
            q2[2] = -0.317392992493
            q2[3] = 0.94829409379

            qr = tf.transformations.quaternion_multiply(q1, q2)

            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "zelle_front_center"
            static_transformStamped.transform.translation.x = x1
            static_transformStamped.transform.translation.y = y1
            static_transformStamped.transform.translation.z = 0

            static_transformStamped.transform.rotation.x = 0
            static_transformStamped.transform.rotation.y = 0
            static_transformStamped.transform.rotation.z = qr[2]
            static_transformStamped.transform.rotation.w = qr[3]
            broadcaster.sendTransform(static_transformStamped)

            rospy.sleep(4)
            print("zelle_front_center frame created")
            movebase_client()

        if theta <= 0:
            print("Right Case 2: TurtleBot is to the right of Right")
            theta_rad = math.radians(abs(theta))
            y11 = round(math.sin(theta_rad) * hypotenuse, 1)
            x11 = round(math.cos(theta_rad) * hypotenuse, 1)
            x1 = x11 + 0.35
            y1 = y11 + 0.4

            print("x1=", x1, "y1=", y1)

            q_current.z = -1 * (q_current.z)
            print(q_current)

            q1 = [None] * 4
            q1[0] = q_current.x
            q1[1] = q_current.y
            q1[2] = q_current.z
            q1[3] = q_current.w

            q2 = [None] * 4
            q2[0] = 0
            q2[1] = 0
            q2[2] = -0.317392992493
            q2[3] = 0.94829409379

            qr = tf.transformations.quaternion_multiply(q1, q2)

            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "base_link"
            static_transformStamped.child_frame_id = "zelle_front_center"
            static_transformStamped.transform.translation.x = x1
            static_transformStamped.transform.translation.y = y1
            static_transformStamped.transform.translation.z = 0

            static_transformStamped.transform.rotation.x = 0
            static_transformStamped.transform.rotation.y = 0
            static_transformStamped.transform.rotation.z = qr[2]
            static_transformStamped.transform.rotation.w = qr[3]
            broadcaster.sendTransform(static_transformStamped)

            rospy.sleep(4)
            print("zelle_front_center frame created")
            movebase_client()


rospy.init_node('end_position_generation')
rospy.Subscriber("hypotenuse", Point, callback)
rospy.Subscriber("side_detected", String, callback2)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback3)
rospy.spin()
