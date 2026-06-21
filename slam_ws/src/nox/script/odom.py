#!/usr/bin/env python


import rospy
import math
import tf2_ros
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32 
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import *
flag = 0

linear_x = 0.0
angular_z = 0.0

dis = 0.0
gocquay = 0.0

def encoder_msgs(data):
	global linear_x,dis,gocquay
	if (data.data[0] == 0 and data.data[1] == 0):
		gocquay = 0.0
		dis = 0.0
	elif (data.data[0]>0 and data.data[1]>0):
		gocquay = 0.0
		encorder_average = (data.data[0] + data.data[1])/6.0
		dis = (3.14*0.06)*(encorder_average/330.0)*1.5
	elif (data.data[0]<0 and data.data[1]<0):
		gocquay = 0.0
		encorder_average = (data.data[0] + data.data[1])/6.0
		dis = (3.14*0.06)*(encorder_average/330.0)*1.5

	elif (data.data[0]<0 and data.data[1]>0):
		dis = 0.0
		encorder_average = ((-1)*data.data[0] + data.data[1])/10.0
		gocquay = (encorder_average/330.0)*(2*3.14)*0.495
		
	elif (data.data[0]>0 and data.data[1]<0):
		dis = 0.0
		encorder_average = (data.data[0] + (-1)*data.data[1])/10.0
		gocquay = (-1)*(encorder_average/330.0)*(2*3.14)*0.495
		

	


if __name__ == '__main__':
	rospy.init_node('tf_broad_caster')
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	
	t.header.frame_id = "odom"
	t.child_frame_id = "base_link"
	pose = Pose()
	pose.position.x = 0.0
	pose.orientation.w = 1.0
	rate = rospy.Rate(10.0)
	
	while not rospy.is_shutdown():
		delta_x = 0.0
		delta_y = 0.0
		rospy.Subscriber('encoder', Int32MultiArray, encoder_msgs)
		
		roll,pitch,yaw = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
		
		
		
		delta_x =dis * math.cos(yaw)
		delta_y = dis * math.sin(yaw)
			
		pose.position.x = pose.position.x + delta_x
		pose.position.y = pose.position.y + delta_y
		yaw = yaw + gocquay
		pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w  = quaternion_from_euler(roll,pitch,yaw)

		t.header.stamp = rospy.Time.now()
		t.transform.translation.x =  pose.position.x
		t.transform.translation.y = pose.position.y
		t.transform.translation.z = pose.position.z
		t.transform.rotation.x = pose.orientation.x
		t.transform.rotation.y = pose.orientation.y
		t.transform.rotation.z =pose.orientation.z
		t.transform.rotation.w = pose.orientation.w
		br.sendTransform(t)
		rate.sleep()
    

