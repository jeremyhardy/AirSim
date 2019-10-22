﻿#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
sys.path.insert(0, '../ros/')

import setup_path
import airsim
import os
import rospy
import cv2
import math
import utils
import subprocess
import time
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, Range, Imu, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
import img_publisher as img
import imu_publisher as inertial
import pc_publisher

 
# gravity = airsim.Vector3r(0, 0, 9.81)
# Path where the bag file from the simulation will be saved
path = '/home/sim/data'

# Mono camera gate
MONO = False

# Stereo camera gate: downward facing through simSetCameraOrientation
STEREO = False

# FLASH - imaging lidar from depthPerpective or depthPlanner image
FLASH = False

# SCANNING - pointcloud2 from the point cloud generated by airsim API
SCANNING = True

# Altimeter grabbed from flash LiDAR image
ALTIMETER = False

# Should the IMU be true data? - Not currently used
TRUTH_IMU = False

# Image collection skips all the getImage framework if there is no use for the data
if MONO == False and STEREO == False and FLASH == False and ALTIMETER == False:
    IMAGE_COLLECTION = False
else:
    IMAGE_COLLECTION = True

# Down is the orientation assigned to the stereo cameras
down = airsim.Quaternionr(0, -0.7071067690849304, 0, 0.7071067094802856)

client = airsim.VehicleClient()
client.confirmConnection()
client.simPause(True)

if IMAGE_COLLECTION:
    # Camera Information
    camera = client.simGetCameraInfo('3')
    test_camera = client.simGetCameraInfo('1')
    if test_camera.pose != down:
        client.simSetCameraOrientation('1', down)
        client.simSetCameraOrientation('2', down)
    FoV = 90

# Coordinate transformation
enu_2_ned = utils.qnorm(airsim.Quaternionr(0.7071068, 0.7071068, 0, 0))
#enu_2_ned = utils.qnorm(airsim.Quaternionr(0, 0, 0, 1))


# call to run ros converter that takes depth image and converts to PC
if FLASH:
    command = 'roslaunch depth_to_pc.launch'
    subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
    time.sleep(2)

# call to record all ros channels except for the garbage ones that stem from processes
command = "rosbag record -a -x '/rosout|/move_base_simple/goal|/rosout_agg|/managerify/(.*)|/rectify/(.*)|/rectified/(.*)' /topic __name:=record_node"
subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=path)

# Publishers
rospy.init_node('drone_publisher', anonymous=True)
publisher_pose = rospy.Publisher('/ground_truth/pose', PoseStamped, queue_size=1)
publisher_path = rospy.Publisher('/ground_truth/path', Path, queue_size=1)
publisher_imu = rospy.Publisher('/imu/imu', Imu, queue_size=1)

# Mono, Flash, Altimeter, Stereo, & Scanning Publishers
if MONO:
    publisher_down = rospy.Publisher('/down/image_raw', Image, queue_size=1)
    publisher_down_info = rospy.Publisher('/down/camera_info', CameraInfo, queue_size=1)

if FLASH:
    publisher_depth = rospy.Publisher('/depth/image_raw', Image, queue_size=1)
    publisher_depth_info = rospy.Publisher('/depth/camera_info', CameraInfo, queue_size=1)

if ALTIMETER:
    publisher_alt = rospy.Publisher('/altimeter', Range, queue_size=1)

if STEREO:
    publisher_left = rospy.Publisher('/left/image_raw', Image, queue_size=1)
    publisher_right = rospy.Publisher('/right/image_raw', Image, queue_size=1)
    publisher_left_info = rospy.Publisher('/left/camera_info', CameraInfo, queue_size=1)
    publisher_right_info = rospy.Publisher('/right/camera_info', CameraInfo, queue_size=1)

if SCANNING:
    publisher_scanning = rospy.Publisher('/lidar/scanning', PointCloud2, queue_size=1)
    publisher_transform = rospy.Publisher('/tf', TFMessage, queue_size=1)

# Sleep to make sure ROS process finishes before proceeding
time.sleep(2)

airsim.wait_key('Press Any Key To Continue')

# One time time acquisition
time_start = rospy.Time.now().to_sec()
# kinematics_start = client.simGetGroundTruthKinematics()

sequence = 0
trigger = 1 
msg_path = Path() 

while trigger == 1:
    client.simPause(False)
    acc_time = rospy.Time.now().to_sec()
    sim_time = rospy.Time.from_sec((acc_time - time_start)*0.7 + time_start)
    truth = client.simGetGroundTruthKinematics()
    environment = client.simGetGroundTruthEnvironment()
    print(truth)
    gravity = utils.quat2vec(environment.gravity)
    # truth = client.getMultirotorState()  
    linear_acc, angular_vel = inertial.Truth2Body(truth, gravity) 
    linear_acc = utils.quat2vec(utils.vec2quat(linear_acc).rotate(enu_2_ned))
    angular_vel = utils.quat2vec(utils.vec2quat(angular_vel).rotate(enu_2_ned))
    position = utils.quat2vec(utils.vec2quat(truth.position).rotate(enu_2_ned))
    orientation = utils.mess2quat(truth.orientation).rotate(enu_2_ned)
    msg_pose_stamped = inertial.CreatePoseMessage(position, orientation, sim_time, sequence)
    msg_imu = inertial.CreateImuMessage(orientation, linear_acc, angular_vel, sim_time, sequence)
    msg_path = inertial.CreatePathMessage(msg_path, msg_pose_stamped, sim_time, sequence) 

    publisher_pose.publish(msg_pose_stamped) 
    publisher_imu.publish(msg_imu) 
    publisher_path.publish(msg_path) 

    if IMAGE_COLLECTION:

        if MONO:
            responses = client.simGetImages([airsim.ImageRequest('3', airsim.ImageType.Scene, False, False), airsim.ImageRequest('3', airsim.ImageType.DepthPerspective, True, False)])
            img_down = img.getRGBImage(responses[0])

            msg_down = img.CreateMonoMessage(img_down, sim_time, sequence)
            msg_down_info = img.CreateInfoMessage(img_down, FoV, sim_time, sequence)

            publisher_down.publish(msg_down)
            publisher_down_info.publish(msg_down_info)

        if ALTIMETER or FLASH:
            responses = client.simGetImages([airsim.ImageRequest('3',airsim.ImageType.DepthPerspective, True, False)])
            # responses = client.simGetImages([airsim.ImageRequest("3",airsim.ImageType.DepthPlanner,True,False)])

        if ALTIMETER:
            img_depth = img.getDepthImage(responses[0])
            depth_val = img_depth[img_depth.shape[0] / 2 - 1][img_depth.shape[1] / 2 - 1]
            msg_alt = img.CreateRangeMessage(depth_val, sim_time, sequence)
            publisher_alt.publish(msg_alt)

        if FLASH:
            img_depth = img.getDepthImage(responses[0])
            msg_depth = img.CreateDepthMessage(img_depth, sim_time, sequence)
            msg_depth_info = img.CreateInfoMessage(img_depth, FoV, sim_time, sequence)
            publisher_depth.publish(msg_depth)
            publisher_depth_info.publish(msg_depth_info)

        if STEREO:
            responses = client.simGetImages([airsim.ImageRequest('2',airsim.ImageType.Scene, False, False),airsim.ImageRequest('1', airsim.ImageType.Scene, False, False)])

            img_left = img.getRGBImage(responses[0])
            img_right = img.getRGBImage(responses[1])

            msg_left = img.CreateMonoMessage(img_left, sim_time, sequence)
            msg_right = img.CreateMonoMessage(img_right, sim_time, sequence)
            msg_left_info = img.CreateInfoMessage(img_left, FoV, sim_time, sequence)
            msg_right_info = img.CreateInfoMessage(img_right, FoV, sim_time, sequence)

            publisher_left.publish(msg_left)
            publisher_right.publish(msg_right)
            publisher_left_info.publish(msg_left_info)
            publisher_right_info.publish(msg_right_info)

    if SCANNING:
        lidar_data = client.getLidarData()
        points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))

        try:
            points = np.reshape(points, (int(points.shape[0] / 3), 3))

            scan_msg = pc_publisher.CreatePC2Message(points, sim_time, sequence)
            lidar_tf = img.CreateTFMessage(truth, sim_time, sequence)

            publisher_transform.publish(lidar_tf) 
            publisher_scanning.publish(scan_msg)

        except ValueError: 
            print("Scanning LiDAR skipped due to incomplete data") 

    sequence += 1
subprocess.Popen('rosnode kill -a', stdin=subprocess.PIPE, shell=True, cwd=path)
