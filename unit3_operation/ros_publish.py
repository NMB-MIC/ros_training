#!/usr/bin/env python3
import rospy #import ros library
from std_msgs.msg import String #import message

def func_1(): #create function
    pub_1 = rospy.Publisher('topic_1', String, queue_size=10) # define topic
    talk_1 = "Sawaddee krab" # set data
    pub_1.publish(talk_1) # publish data
    rospy.loginfo(talk_1) # show data on terminal

def main():
    #Create init node
    rospy.init_node('publish', anonymous=True) # Create init node
    rate = rospy.Rate(1) # publish rate unit hertz (1/second)
    while not rospy.is_shutdown(): # Running until ros is on
        func_1() # call function 
        rate.sleep() # wait until next iteration
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
