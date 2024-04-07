#!/usr/bin/env python3
import rospy #import ros library
from std_msgs.msg import String #import message

def callback(msg): # function for receive parameter
    sub = msg.data # assign value into parameter
    rospy.loginfo(sub) # show data on terminal

def main():
    rospy.init_node('subscriber', anonymous=True) # Create init node
    rospy.Subscriber("topic_1", String, callback) # Subscribe topic
    rospy.spin() # Goto next iteration

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass