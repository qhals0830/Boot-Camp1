#!/usr/bin/env python

import rospy
from chassis_msg.msg import LOG_BYTE0, LOG_BYTE1, LOG_BYTE2
from geometry_msgs.msg import *
from tutorial_msgs.msg import Chassis0,Chassis1,Chassis2,gps
import time


class Tutorial:

    def __init__(self):

        self.aliveCount = 0 
        self.Log_Byte0 = None
        self.Log_Byte1 = None
        self.Log_Byte2 = None
        self.GPS = None

        rospy.Subscriber('/LOG_BYTE0',LOG_BYTE0,self.callback_LOG_BYTE0)
        rospy.Subscriber('/LOG_BYTE1',LOG_BYTE1,self.callback_LOG_BYTE1)
        rospy.Subscriber('/LOG_BYTE2',LOG_BYTE2,self.callback_LOG_BYTE2)
        rospy.Subscriber('/filter/positionlla',Vector3Stamped,self.callback_GPS)


        self.pub_chassis0 = rospy.Publisher('/Chassis0',Chassis0,queue_size= 10)
        self.pub_chassis1 = rospy.Publisher('/Chassis1',Chassis1,queue_size= 10)
        self.pub_chassis2 = rospy.Publisher('/Chassis2',Chassis2,queue_size= 10)
        self.pub_gps = rospy.Publisher('/GPS',gps,queue_size= 10)

        while True:

            if (self.Log_Byte0 is not None) and (self.Log_Byte1 is not None) and (self.Log_Byte2 is not None):
                break
            else:
                print('Waiting for Data...')
                time.sleep(0.5)

        while True:

            if (self.GPS is not None):
                break
            else:
                print('Waiting for Data...')
                time.sleep(0.5)

        print('Subscribe!')

        self.main()


    def main(self):

        r = rospy.Rate(20)

        while not rospy.is_shutdown():

            start_time = time.time()

            print("Publish!")
            self.publish_chassis0(self.Log_Byte0)
            self.publish_chassis1(self.Log_Byte1)
            self.publish_chassis2(self.Log_Byte2)
            self.publish_gps(self.GPS) 

            end4_time = time.time()-start_time
            # print("publish : ", end4_time)
            
            self.aliveCount += 1
            r.sleep()

                
    def callback_LOG_BYTE0(self,msg):

        self.Log_Byte0 = msg

    def callback_LOG_BYTE1(self,msg):

        self.Log_Byte1 = msg

    def callback_LOG_BYTE2(self,msg):
        self.Log_Byte2 = msg

    def callback_GPS(self,msg):
        self.GPS = msg


    def publish_chassis0(self,Log_Byte0):

        chassis0_msg = Chassis0()
        chassis0_msg.header.frame_id = 'world'
        chassis0_msg.header.seq = self.aliveCount
        chassis0_msg.header.stamp = rospy.Time.now()
        
        chassis0_msg.Wheel_Speed_FL = Log_Byte0.WHL_SPD_FL
        chassis0_msg.Wheel_Speed_FR= Log_Byte0.WHL_SPD_FR
        chassis0_msg.Wheel_Speed_RR= Log_Byte0.WHL_SPD_RR
        chassis0_msg.Wheel_Speed_RL= Log_Byte0.WHL_SPD_RL

        self.pub_chassis0.publish(chassis0_msg)


    def publish_chassis1(self,Log_Byte1):

        chassis1_msg = Chassis1()
        chassis1_msg.header.frame_id = 'world'
        chassis1_msg.header.seq = self.aliveCount
        chassis1_msg.header.stamp = rospy.Time.now()
        
        chassis1_msg.Lat_Acceleration = Log_Byte1.LAT_ACCEL
        chassis1_msg.Long_Acceleration= Log_Byte1.LONG_ACCEL
        chassis1_msg.Yaw_Rate= Log_Byte1.YAW_RATE

        self.pub_chassis1.publish(chassis1_msg)



    def publish_chassis2(self,Log_Byte2):

        chassis2_msg = Chassis2()
        chassis2_msg.header.frame_id = 'world'
        chassis2_msg.header.seq = self.aliveCount
        chassis2_msg.header.stamp = rospy.Time.now()
        
        chassis2_msg.Steering_Angle = Log_Byte2.SAS_Angle
        chassis2_msg.Steering_Speed= Log_Byte2.SAS_Speed


        self.pub_chassis2.publish(chassis2_msg)

    def publish_gps(self,GPS):

        gps_msg = gps()
        gps_msg.header.frame_id = 'world'
        gps_msg.header.seq = self.aliveCount
        gps_msg.header.stamp = rospy.Time.now()
        
        gps_msg.longitude = GPS.vector.x
        gps_msg.latitude= GPS.vector.y
        gps_msg.altitude= GPS.vector.z


        self.pub_gps.publish(gps_msg)


if __name__ == '__main__':

    try:

        rospy.init_node('Tutorial')
        Tutorial()
        rospy.spin()

    except rospy.ROSInterruptException:  # ROSInterruptException exception, which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. 
        pass