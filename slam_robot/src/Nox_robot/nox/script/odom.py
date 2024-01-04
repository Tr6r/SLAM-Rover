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
"""
def linear_x_callback(data):
	global flag
	if flag == 0:
		global linear_x
		if data.data == 0:
			linear_x = 0;
			flag = 1
		elif data.data > 0:
			#linear_x = data.data - 0.2556
			linear_x = data.data 
			flag = 1
		elif data.data < 0:
			#linear_x = data.data + 0.27
			linear_x = data.data
			flag = 1

def angular_z_callback(data):
	global flag 
	if flag == 1 :
		global angular_z
		global linear_x
		if data.data == 0:
			angular_z = 0;
		elif data.data >0:
			#angular_z = data.data * 0.1 - 0.025
			angular_z = data.data
			linear_x = 0
		elif data.data <0:
			#angular_z = data.data * 0.1 + 0.025
			angular_z = data.data
			linear_x = 0
		flag = 0"""
'''
def pose_callback(data):
	global flag,linear_x,angular_z,pose,gocquay
	
	if not data.transforms:
		return
	last_transform = data.transforms[-1]
	print(last_transform)
	position = last_transform.transform.translation
	pose.position.x = position.x
	pose.position.y = position.y
	pose.position.z = position.z
	
	orientation = last_transform.transform.rotation
	
	pose.orientation.x = orientation.x
	
	pose.orientation.y = orientation.y
	pose.orientation.z = orientation.z
	pose.orientation.w = orientation.w
	
	roll,pitch,yaw = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
	if flag == 1 :
		if angular_z != 0:
			linear_x = 0
		delta_x = linear_x * math.cos(yaw)
		delta_y = linear_x * math.sin(yaw)

		pose.position.x +=delta_x
		pose.position.y +=delta_y
		gocquay = gocquay+angular_z
		pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w  = quaternion_from_euler(roll,pitch,gocquay)
		flag = 0'''

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
    

