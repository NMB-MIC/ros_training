#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion


rospy.loginfo(["connected"])
rospy.init_node('goal_nav', anonymous=True)
move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
rate = rospy.Rate(20) # 5 hz
rospy.loginfo("wait for the action server to come up")
move_base.wait_for_server(rospy.Duration(5))
time_out = 120

def get_values():
    global x,y,theta
    user_input = input("put goal position a, b or c: ").strip().lower()
    if user_input == 'a':
        x, y, theta = 1.628, -1.543, 1.469
    elif user_input == 'b':
        x, y, theta = -1.104, 1.790, -2.074
    elif user_input == 'c':
        x, y, theta = -1.910, -0.465, -0.015
    else:
        print("Please type a,b or c not: "+user_input)
        return get_values()  # เรียกฟังก์ชันใหม่ถ้าไม่ถูกต้อง
    rospy.loginfo(["Going to : " + "x: " + str(x) + " y: " + str(y) + " theta: " + str(theta)])

def movebase_client():
    #convert euler to quanternion
    q = quaternion_from_euler(0,0,theta)

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))

    move_base.send_goal(goal)
    wait = move_base.wait_for_result(rospy.Duration(time_out))
    state = move_base.get_state()
    if wait and state == GoalStatus.SUCCEEDED:
        result_stat  = "OK"
    else:
        move_base.cancel_goal()
        result_stat  = "Fail"
    rospy.loginfo(["Done result : " + result_stat])

def main():
    while not rospy.is_shutdown():
        get_values()
        movebase_client()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
