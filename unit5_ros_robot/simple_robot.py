#!/usr/bin/env python3
import rospy
from threading import Thread
from sensor_msgs.msg import JointState
from dobot_api import dobot_api_dashboard,dobot_api_feedback,MyType
import time
import numpy as np
import math
import threading

dobot_Enable = True

joint1 = 0
joint2 = 0
joint3 = 0
joint4 = 0

j_status = []

# Initialize the node
rospy.init_node("joint_publish_dobot",anonymous=True)
# Publisher robot
publisherRobot = rospy.Publisher("Joint_states",JointState,queue_size=10)
# Get parameter when start launch ex. ip:=192.168.1.xxx
# mg400_ip = rospy.get_param('~ip_mg400')
mg400_ip = "192.168.1.6"

# Rate controller [unit Hz]
rate_control =rospy.Rate(30) #default 100

# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard(mg400_ip, 29999)
client_feedback = dobot_api_feedback(mg400_ip, 30003)

client_dashboard.DisableRobot()
time.sleep(1)
# Remove alarm
client_dashboard.ClearError()
time.sleep(0.5)

# Description The upper function was enabled successfully
client_dashboard.EnableRobot()
time.sleep(0.5)

# Select user and Tool coordinate system 0
client_dashboard.User(0)
client_dashboard.Tool(0)

# Feedback information display port 30003
def mg400_feedback(threaName):
    global dobot_Enable,client_feedback
    global joint1,joint2,joint3,joint4
    global x,y,z
    while not rospy.is_shutdown():
        time.sleep(0.05) 
        all = client_feedback.socket_feedback.recv(10240) 
        data = all[0:1440]
        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':           
            #print("============== Feed Back ===============")
            
            MG400_endpoint =  np.around(a['Tool_vector_target'], decimals=4)[0]
            #print("MG400_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [r:{3}] , [user:{4}] , [tool:{5}]".format(MG400_endpoint[0],MG400_endpoint[1],MG400_endpoint[2],MG400_endpoint[3],MG400_endpoint[4],MG400_endpoint[5]))
            
            MG400_joint = np.around(a['q_target'], decimals=4)[0]
            #print("MG400_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}]".format(MG400_joint[0],MG400_joint[1],MG400_joint[2],MG400_joint[3]))
            #print("========================================")

        joint1 = MG400_joint[0]
        joint2 = MG400_joint[1]
        joint3 = MG400_joint[2]
        joint4 = MG400_joint[3]
        # Variables
        msg = JointState()
        msg.header.frame_id = ""
        msg.name = ["j1","j2_1","j2_2","j3_1","j3_2","j4_1","j4_2","j5"]    # Joint name (array assignment)
        #msg.name = ["joint1","joint2","joint3","joint4"]
        
        global j1,j2,j3,j4

        j1 = joint1*math.pi / 180
        j2 = joint2*math.pi / 180
        j3 = joint3*math.pi / 180
        j4 = joint4*math.pi / 180
        # Initialize the time of publishing
        msg.header.stamp=rospy.Time.now()
            # Joint angle values
        #msg.position = [j1, j2, j2, j3-j2, -j2, -j3, j3, j4]
        msg.position = [joint1,joint2,joint3,joint4]
            # Publish message
        j_status = [joint1,joint2,joint3,joint4]
        
        publisherRobot.publish(msg)
            # Increase sequence
        msg.header.seq += 1      
        rate_control.sleep()

thread_robot1 = threading.Thread(target=mg400_feedback,args=("Thread-1",))
thread_robot1.start()

def program1():
    client_feedback.JointMovJ(0,0,30,0,0,0)

def program2():
    client_feedback.JointMovJ(0,30,30,0,0,0)
try:
    while(not rospy.is_shutdown()) and dobot_Enable == True:    
        client_dashboard.SpeedJ(10)     
        program1()
        time.sleep(3)
        program2()
        time.sleep(3)
        client_dashboard.DO(2,1) 
        time.sleep(1)  
        client_dashboard.DO(2,0) 
        time.sleep(1)
        
except KeyboardInterrupt or rospy.ROSInternalException:
            dobot_Enable = False
            client_dashboard.DisableRobot()
            rospy.logfatal("Node crashed due to an internal exception")

client_dashboard.DisableRobot()
client_dashboard.close()
client_feedback.close()