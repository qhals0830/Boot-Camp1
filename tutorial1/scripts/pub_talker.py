#!/usr/bin/env python
# Software License Agreement (BSD License)
#

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String 

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10) #rospy.Publisher('topic name','message type','queue_size')
    rospy.init_node('pub_talker', anonymous=True) # it tells rospy the name of your node , anonymous = True ensures that your node has a unique name
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():  # check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise)
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep() # rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop.

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:  # ROSInterruptException exception, which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. 
        pass
