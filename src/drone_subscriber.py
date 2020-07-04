#! /usr/bin/env python

import rospy
import tf
from dronekit_publish.msg import Altitude, Attitude, Local_Frame

att = [0,0,0]
loc = [0,0,0]
init_yaw = 0
flag = True

def attitude_callback(msg):
    global att, init_yaw, flag
    att[0] = Attitude.roll
    att[1] = Attitude.pitch
    att[2] = Attitude.yaw

    if flag:
        init_yaw = att[2]
        flag = False

    rospy.loginfo(msg)

def local_frame_callback(msg):
    global loc
    loc[0] = Local_Frame.x
    loc[1] = Local_Frame.y
    loc[2] = Local_Frame.z

    rospy.loginfo(msg)

def rangefinder_callback(msg):
    global rf
    rf[0] = Altitude.distance
    rf[1] = Altitude.voltage

    rospy.loginfo(msg)

def listener():
    subscribe_att = rospy.Subscriber("/attitude", Attitude, attitude_callback)
    subscribe_loc = rospy.Subscriber("/local_frame", Local_Frame, local_frame_callback)
    subscribe_alt = rospy.Subscriber("/rangefinder", Altitude, rangefinder_callback)

    br = tf.TransformBroadcaster()
    br.sendTransform((loc[0], loc[1], rf[1]), 
                      tf.transformations.quaternion_from_euler(att[0], att[1], att[2]-init_yaw), 
                      rospy.Time.now(), 
                      '/base_link', 
                      '/map')

    br2 = tf.TransformBroadcaster()
    br2.sendTransform((0, 0, 0.2), 
                      tf.transformations.quaternion_from_euler(0, 0, 1.5708), 
                      rospy.Time.now(), 
                      '/base_link', 
                      '/laser')
    
    #keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node('drone_listener', anonymous=True)
    rospy.Subscriber('power', String, tel, queue_size=10)

    listener()

    # while not rospy.is_shutdown:
    #     br = tf.TransformBroadcaster()
    #     br.sendTransform(loc[0], loc[1], loc[2], tf.transformations.quaternion_from_euler(att[0], att[1], att[2]-init_yaw), rospy.Time.now(), '/base_link', '/map')