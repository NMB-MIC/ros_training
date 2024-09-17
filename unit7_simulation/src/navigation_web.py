#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import String, Float32
import mysql.connector
import time
from datetime import datetime
import signal
import sys
from math import atan2, radians

#-----------------------------------------------Setting for connection-----------------------------------------------#
global move_base, client, connection, start_point, finish_point, start_time, finish_time, duration, result_stat, goal_point, goal_x, goal_y, goal_th, distance, start_distance, end_distance, temp_distance, battery, start_battery, end_battery, temp_battery
start_point = "HOME"
rospy.init_node('nav_goal', anonymous=False)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
feedbackPub = rospy.Publisher("/robot_feedback_topic", String, queue_size=10)
rate = rospy.Rate(5)  # 5 hz
rospy.loginfo("wait for the action server to come up")
move_base.wait_for_server(rospy.Duration(5))
time_out = 200  # time out if AGV cannot reach goal

#----------------------------Connect MySQL
try:
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )
    if connection.is_connected():
        rospy.loginfo("Successfully connected to MySQL database")
    else:
        rospy.logwarn("Failed to connect to MySQL database")
        sys.exit()
except mysql.connector.Error as err:
    rospy.logwarn("Failed to connect to MySQL database: {}".format(err))
    sys.exit()

cursor = connection.cursor()
cursor.close()

#-----------------------------------------------Signal Handler-----------------------------------------------#
def signal_handler(sig, frame):
    rospy.loginfo('Exiting the program...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#-----------------------------------------------Main Function for Navigation-----------------------------------------------#
def goal_callback(msg):
    global goal_point, start_distance, end_distance, start_battery
    goal_point = msg.data
    rospy.loginfo("Subscribed Goal is: {}".format(goal_point))

    # Subscribe to distance and battery topics
    rospy.Subscriber('/distance_topic', Float32, distance_callback)
    rospy.Subscriber('/battery_topic', Float32, battery_callback)

    rospy.loginfo("Waiting for start distance subscription")
    rospy.wait_for_message('/distance_topic', Float32)
    start_distance = temp_distance
    
    rospy.wait_for_message('/battery_topic', Float32)
    start_battery = temp_battery
    rospy.loginfo("Start distance is: {}".format(start_distance))
    rospy.loginfo("Start battery is: {}".format(start_battery))

    check_database()
    # rotate_direction()
    movebase_client()
    save_database()

def distance_callback(msg):
    global temp_distance
    temp_distance = msg.data

def battery_callback(msg):
    global temp_battery
    temp_battery = msg.data

def check_database():
    global goal_point, goal_x, goal_y, goal_th
    rospy.loginfo("Checking information in database")
    cursor = connection.cursor()
    select_query = """
        SELECT goal_x, goal_y, goal_th FROM goal_tb WHERE goal_point = %s
    """
    cursor.execute(select_query, (goal_point,))
    result = cursor.fetchone()
    cursor.close()
    if result:
        goal_x, goal_y, goal_th = result
        rospy.loginfo("Found goal information in database - X: {}, Y: {}, Theta: {}".format(goal_x, goal_y, goal_th))
    else:
        rospy.logwarn("Goal information not found in database for goal point: {}".format(goal_point))
        return

def amcl_pose_callback(data):
    direction = atan2(goal_y - data.pose.pose.position.y, goal_x - data.pose.pose.position.x)
    direction_radians = radians(direction)
    q = quaternion_from_euler(0, 0, direction_radians)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(data.pose.pose.position.x, data.pose.pose.position.y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
    move_base.send_goal(goal)
    wait = move_base.wait_for_result(rospy.Duration(20))
    state = move_base.get_state()
    if wait and state == GoalStatus.SUCCEEDED:
        movebase_client()
        save_database()
    else:
        move_base.cancel_goal()

def rotate_direction():
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

def movebase_client():
    global start_time, finish_time, finish_point, result_stat, duration, distance, start_distance, end_distance, end_battery, battery, start_battery, goal_th, goal_x, goal_y
    q = quaternion_from_euler(0, 0, goal_th)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(goal_x, goal_y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
    start_time = datetime.now()
    start_time_millis = time.time()
    move_base.send_goal(goal)
    rospy.loginfo("Going to goal: {}".format(goal_point))
    wait = move_base.wait_for_result(rospy.Duration(time_out))
    state = move_base.get_state()
    if wait and state == GoalStatus.SUCCEEDED:
        result_stat = "OK"
        finish_time = datetime.now()
        finish_point = goal_point
        duration = int(round(time.time() - start_time_millis))
    else:
        move_base.cancel_goal()
        result_stat = "NG"
        finish_time = datetime.now()
        finish_point = goal_point
        duration = int(round(time.time() - start_time_millis))

    rospy.loginfo("Waiting for end distance subscription")
    rospy.wait_for_message('/distance_topic', Float32)
    end_distance = temp_distance
    rospy.loginfo("Start distance is: {}".format(start_distance))
    rospy.loginfo("End distance is: {}".format(end_distance))
    distance = end_distance - start_distance

    rospy.loginfo("Distance: {}".format(distance))
    rospy.wait_for_message('/battery_topic', Float32)
    end_battery = temp_battery
    rospy.loginfo("Start battery is: {}".format(start_battery))
    rospy.loginfo("End battery is: {}".format(end_battery))
    battery = end_battery - start_battery
    rospy.loginfo("Battery: {}".format(battery))
    
    feedbackPub.publish(String(data=result_stat))

def save_database():
    global start_point
    rospy.loginfo("Saving information in database")
    cursor = connection.cursor()
    dates = datetime.now().strftime("%Y-%m-%d")
    times = datetime.now().strftime("%H:%M:%S")
    insert_query = """
        INSERT INTO record_tb
        (Date, Time, Start_Point, Finish_Point, Result_, Start_time, Finish_time, Duration, Distance, Battery_remain, Battery_usage) 
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
    """
    cursor.execute(insert_query, (dates, times, start_point, finish_point, result_stat, start_time, finish_time, duration, distance, end_battery, battery))
    connection.commit()
    cursor.close()
    rospy.loginfo("Recorded in database - Start Point: {}, Finish Point: {}, Result: {}, Start Time: {}, Finish Time: {}, Duration: {}, Distance: {}, Battery_remain: {}, Battery_usage: {}".format(start_point, finish_point, result_stat, start_time, finish_time, duration, distance, end_battery, battery))
    if result_stat == "OK":
        start_point = finish_point
    else:
        start_point = "HOME"
    return

#----------------------------------------------------------------Main-------------------------------------------------------------#
def main():
    while not rospy.is_shutdown():
        rospy.loginfo("Waiting for sub goal")
        rospy.wait_for_message('/robot_target_topic', String)
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.Subscriber('/robot_target_topic', String, goal_callback)
        main()
    except rospy.ROSInterruptException:
        cursor.close()
        connection.close()
        rospy.loginfo("Navigation is Closed")
