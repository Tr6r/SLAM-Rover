#!/usr/bin/env python

import rospy
import math
import tf2_ros
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32 
from geometry_msgs.msg import *
flag = 0
linear_x = 0.0
angular_z = 0.0
gocquay = 0.0

pose = Pose()
pose.position.x = 2.0
pose.orientation.w = 1.0

rospy.init_node('tf_broad_caster')
rate = rospy.Rate(10.0)
br = tf2_ros.TransformBroadcaster()
t = TransformStamped()
	
t.header.frame_id = "odom"
t.child_frame_id = "base_link"

t.header.stamp = rospy.Time.now()
t.transform.translation.x =  pose.position.x
t.transform.translation.y = pose.position.y
t.transform.translation.z = pose.position.z
t.transform.rotation.x = pose.orientation.x
t.transform.rotation.y = pose.orientation.y
t.transform.rotation.z =pose.orientation.z
t.transform.rotation.w = pose.orientation.w
rospy.sleep(0.3)
br.sendTransform(t)

def linear_x_callback(data):
	global flag 
	if flag == 0:
		global linear_x
		if data.data == 0:
			linear_x = 0;
		elif data.data > 0:
			linear_x = data.data - 0.25
		elif data.data < 0:
			linear_x = data.data + 0.25
		flag = 1
def angular_z_callback(data):
	global flag 
	if flag == 1 :
		global angular_z
		global linear_x
		if data.data == 0:
			angular_z = 0;
		elif data.data !=0:
			angular_z = data.data * 0.1 + 0.03
			linear_x = 0
def pose_callback(data):
	global flag,linear_x,angular_z,gocquay,br,t
	tf2_pose = Pose()
	if not data.transforms:
		return
	last_transform = data.transforms[-1]
	position = last_transform.transform.translation
	tf2_pose.position.x = position.x
	tf2_pose.position.y = position.y
	tf2_pose.position.z = position.z
		
	orientation = last_transform.transform.rotation
	tf2_pose.orientation.x = orientation.x
	tf2_pose.orientation.y = orientation.y
	tf2_pose.orientation.z = orientation.z
	tf2_pose.orientation.w = orientation.w
		
	roll,pitch,yaw = euler_from_quaternion([tf2_pose.orientation.x,tf2_pose.orientation.y,tf2_pose.orientation.z,tf2_pose.orientation.w])
	if angular_z != 0:
		linear_x = 0
	delta_x =linear_x * math.cos(yaw)
	delta_y = linear_x * math.sin(yaw)

	tf2_pose.position.x +=delta_x
	tf2_pose.position.y +=delta_y
	gocquay = gocquay+angular_z
	tf2_pose.orientation.x,tf2_pose.orientation.y,tf2_pose.orientation.z,tf2_pose.orientation.w  = quaternion_from_euler(roll,pitch,gocquay)
	t.header.stamp = rospy.Time.now()
	t.transform.translation.x =  tf2_pose.position.x
	t.transform.translation.y = tf2_pose.position.y
	t.transform.translation.z = tf2_pose.position.z
	t.transform.rotation.x = tf2_pose.orientation.x
	t.transform.rotation.y = tf2_pose.orientation.y
	t.transform.rotation.z =tf2_pose.orientation.z
	t.transform.rotation.w = tf2_pose.orientation.w
	br.sendTransform(t)


while not rospy.is_shutdown():
	rospy.Subscriber('linear_x', Float32, linear_x_callback)
	rospy.Subscriber('angular_z', Float32, angular_z_callback)
	rospy.Subscriber('tf', TFMessage, pose_callback)	
	rate.sleep()
    

	


