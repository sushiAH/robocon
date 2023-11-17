#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
import tf2_ros
import tf_conversions
import math
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

def service_client(ID):
    
    
    rospy.wait_for_service("/get_velocity")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        getvelocity = rospy.ServiceProxy("/get_velocity",Getvelocity)
        resp1 = getvelocity(ID)

        print(resp1.velocity)

        return ID,resp1.velocity

        rate.sleep()

def main(ID,vel):
    d = 10
    r = 5
    pi = 3.1415
    previous_x = 0
    previous_y = 0
    previous_theta = 0
    Vr = 0
    Vl = 0


    rate = rospy.Rate(10)
    currdent_time = rospy.get_time()
    last_time = rospy.get_time()

    br = tf2_ros.TransformBroadcaster()

    pub_odom = rospy.Publisher("odom",Odometry,queue_size = 10)

    while not rospy.is_shutdown():

        current_time = rospy.get_time()
        dt = (current_time - last_time)
        if ID == 1:
            Vl = vel
        elif ID == 2:
            Vr = vel
            
        delta_Ll = Vl*dt
        delta_Lr = Vr*dt
        delta_L = (delta_Ll + delta_Lr)/2
        delta_theta = (delta_Lr - delta_Ll)/2*d
        theta = previous_theta + delta_L*dt
        x_dot = delta_L*math.cos(theta)
        y_dot = delta_L*math.sin(theta)

        x = previous_x + x_dot*dt
        y = previous_y + y_dot*dt

        q = tf_conversions.transformations.quaternion_from_euler(0,0,theta)

        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = delta_L
        odom_msg.twist.twist.angular.z = delta_theta 

        pub_odom.publish(odom_msg)


        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q =  tf_conversions.transformations.quaternion_from_euler(0,0,theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

        print(x)
        print(y)
        print(theta)

        previous_theta = theta
        previous_x = x
        previous_y = y

        

        rate.sleep()







if __name__ == '__main__':
    while not rospy.is_shutdown():

        try:
            rospy.init_node("odometry")

            result1 = service_client(1)
            result2 = service_client(2)


            main(*result1)
            main(*result2)
        

        except rospy.ROSInterruptException:
            pass
    

    
