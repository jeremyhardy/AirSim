import setup_path, rospy, airsim, cv2, utils
from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2, PointField, Range
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, TwistStamped, AccelStamped, Point, Pose, Twist 
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import numpy as np

def CreatePoseMessage(position, orientation, gt_time, sequence):
    msg_pose_stamped = PoseStamped()
    msg_point = Point() 
    msg_quat = Quaternion()
    msg_pose = Pose() 
    msg_pose_stamped.header.frame_id = "world" 
    msg_pose_stamped.header.stamp = gt_time
    msg_pose_stamped.header.seq = sequence
    msg_point.x = position.x_val
    msg_point.y = position.y_val
    msg_point.z = position.z_val
    msg_quat.w = orientation.w_val
    msg_quat.x = orientation.x_val
    msg_quat.y = orientation.y_val
    msg_quat.z = orientation.z_val
    msg_pose.position = msg_point
    msg_pose.orientation = msg_quat
    msg_pose_stamped.pose = msg_pose
    return msg_pose_stamped 

def CreateTwistMessage(linear, angular, gt_time, sequence):
    msg_av = Vector3()
    msg_lv = Vector3() 
    msg_twist = Twist()
    msg_twist_stamped = TwistStamped()
    msg_twist_stamped.header.frame_id = "world" 
    msg_twist_stamped.header.stamp = gt_time
    msg_twist_stamped.header.seq = sequence
    msg_lv.x = linear.x_val
    msg_lv.y = linear.y_val
    msg_lv.z = linear.z_val
    msg_twist.linear = msg_lv
    msg_av.x = angular.x_val 
    msg_av.y = angular.y_val 
    msg_av.z = angular.z_val 
    msg_twist.angular = msg_av
    msg_twist_stamped.twist = msg_twist
    return msg_twist_stamped 

def CreateImuMessage(orientation, linear_acceleration, angular_velocity, imu_time, sequence):
    msg_av = Vector3()
    msg_la = Vector3()
    msg_quat = Quaternion()
    msg_imu = Imu() 
    msg_imu.header.stamp = imu_time
    msg_imu.header.frame_id = "body_frame"
    msg_imu.header.seq = sequence
    msg_quat.x = orientation.x_val 
    msg_quat.y = orientation.y_val 
    msg_quat.z = orientation.z_val 
    msg_quat.w = orientation.w_val 
    msg_imu.orientation = msg_quat 
    msg_av.x = angular_velocity.x_val 
    msg_av.y = angular_velocity.y_val 
    msg_av.z = angular_velocity.z_val 
    msg_imu.angular_velocity = msg_av 
    msg_la.x = linear_acceleration.x_val
    msg_la.y = linear_acceleration.y_val 
    msg_la.z = linear_acceleration.z_val 
    msg_imu.linear_acceleration = msg_la
    return msg_imu 








