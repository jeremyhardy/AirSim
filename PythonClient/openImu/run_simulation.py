import sys, pdb
sys.path.insert(0, '../ros/')

import setup_path, airsim, os, rospy, cv2, math, subprocess, utils, time 
import pandas as pd
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Range, Imu
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
import img_publisher as img
import imu_publisher as inertial
import read_data as traj

path = '/home/sim/data'

MONO   = True 
STEREO = False
LIDAR  = True 
TRUTH_IMU = False
minimum_altitude = 0

if(MONO == False and STEREO == False and LIDAR == False): 
    IMAGE_COLLECTION = False
else: 
    IMAGE_COLLECTION = True 
    
down = airsim.Quaternionr(0,-7071067690849304,0,0.7071067094802856)
imu  = utils.qnorm(airsim.Quaternionr(1,0,0,0),10)

if(IMAGE_COLLECTION): 
    client = airsim.VehicleClient()
    client.confirmConnection()
    
    # Camera Information  
    camera = client.simGetCameraInfo("3")
    test_camera = client.simGetCameraInfo("1") 
    if(test_camera.pose!=down):
        client.simSetCameraOrientation("1", down)
        client.simSetCameraOrientation("2", down)
    FoV = 90
    
# Coordinate transformation     
enu_2_ned = utils.qnorm(airsim.Quaternionr(0.7071068,0.7071068,0,0),10)

# panda read csv 
position, attitude, accelerometer, gyroscope, noisy_accelerometer, noisy_gyroscope, time = traj.readData('/home/tigerteam/gnss-ins-sim/demo_saved_data/openIMU_landing/')

# Get position and attitude data from panda read
ned_position_data = traj.getPosition(position)
ned_quaternion_data = traj.getAttitude(attitude)
time_new = traj.getTime(time)

# Gate for including simulated IMU data or perfect IMU readings 
if(TRUTH_IMU):
    print("Perfect IMU Data Selected") 
    ned_accel = traj.getAccel(accelerometer)
    ned_gyro = traj.getGyro(gyroscope)
else:
    print("Noisy IMU Data Selected") 
    ned_accel = traj.getNoisyAccel(noisy_accelerometer)
    ned_gyro = traj.getNoisyGyro(noisy_gyroscope)
    
# call to run ros converter that takes depth image and converts to PC 
command = "roslaunch depth_to_pc.launch"
subprocess.Popen(command , stdin=subprocess.PIPE, shell=True)
command = "rosbag record -a -x '/rosout|/move_base_simple/goal|/rosout_agg|/managerify/(.*)|/rectify/(.*)|/rectified/(.*)' /topic __name:=record_node"
subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=path)

# Sleep to make sure ROS process finishes before proceeding 
time.sleep(2) 
            
# Publishers
rospy.init_node('image_publisher',anonymous=True)
publisher_pose  = rospy.Publisher('/ground_truth/pose', PoseStamped, queue_size=1)
publisher_imu   = rospy.Publisher('/imu/imu', Imu, queue_size=1)

if(MONO): 
    publisher_down  = rospy.Publisher('/down/image_raw', Image, queue_size=1)
    publisher_down_info = rospy.Publisher('/down/camera_info', CameraInfo, queue_size=1)
    
if(LIDAR): 
    publisher_depth = rospy.Publisher('/depth/image_raw', Image, queue_size=1)
    publisher_depth_info  = rospy.Publisher('/depth/camera_info', CameraInfo, queue_size=1)
    publisher_alt   = rospy.Publisher('/altimeter', Range, queue_size=1)
    
if(STEREO): 
    publisher_left = rospy.Publisher('/left/image_raw', Image, queue_size=1)
    publisher_right = rospy.Publisher('/right/image_raw', Image, queue_size=1)
    publisher_left_info = rospy.Publisher('/left/camera_info', CameraInfo, queue_size=1)
    publisher_right_info = rospy.Publisher('/right/camera_info', CameraInfo, queue_size=1)

airsim.wait_key('Data Processed & ROS is ready... Press Any Key To Continue')

# One time time acquisition 
time_start = rospy.Time.now().to_sec()

final = len(ned_position_data) - 1

# iterative quaternion normalization becuase rotation and __mult__ tools are picky as hellll 
#rotation_start = utils.qnorm(quaternion_data[0].conjugate(),10) 
ned_rotation_final_conj = utils.qnorm(ned_quaternion_data[final].conjugate(),10)
#enu_rotation_first_conj = utils.qnorm(enu_quaternion_data[0].conjugate(),10)

ned_final_position = ned_position_data[final] 

sequence = 0
for i in range(len(ned_position_data)):
    print(i, len(ned_position_data))
    
    quaternion_astronav = ned_quaternion_data[i].rotate(enu_2_ned)
    position_astronav = utils.vec2quat(ned_position_data[i]).rotate(enu_2_ned) #airsim.Vector3r(ned_position_data[i].y_val, ned_position_data[i].x_val, -ned_position_data[i].z_val)
    
    position_airsim = ned_position_data[i].__sub__(ned_final_position)
    quaternion_airsim = ned_quaternion_data[i].__mul__(ned_rotation_final_conj)
    position_unreal = airsim.Vector3r(position_airsim.x_val, position_airsim.y_val, position_airsim.z_val-minimum_altitude)

    # Simulation time is the simulation start time (rospy.Time.now().to_sec()) plus the time assigned to this pose in the data sheet. 
    # note that this is not an accurate representation of absolute time, but relative time remains consistent 
    sim_time  = rospy.Time.from_sec(time_new[i] + time_start)
    
    imu_accel = ned_accel[i].rotate(enu_2_ned)
    imu_gyro = ned_gyro[i].rotate(enu_2_ned)
    
    # write ground truth and imu messages 
    msg_pose_stamped = inertial.CreatePoseMessage(position_astronav, quaternion_astronav, sim_time, sequence)
    msg_imu = inertial.CreateImuMessage(quaternion_astronav, imu_accel, imu_gyro, sim_time, sequence)
    
    publisher_pose.publish(msg_pose_stamped)
    publisher_imu.publish(msg_imu)
    
    if(IMAGE_COLLECTION and i%10==0): 

        # set vehicle based on the airsim position and orientation 
        client.simSetVehiclePose(airsim.Pose(position_unreal, quaternion_airsim), True)
        
        if(MONO): 
            # get required imagery from downward facing camera 
            responses = client.simGetImages([airsim.ImageRequest("3",airsim.ImageType.Scene,False,False),airsim.ImageRequest("3",airsim.ImageType.DepthPerspective,True,False)])
            #responses = client.simGetImages([airsim.ImageRequest("3",airsim.ImageType.Scene,False,False),airsim.ImageRequest("3",airsim.ImageType.DepthPlanner,True,False)])
            
            img_down  = img.getRGBImage(responses[0])
            img_depth = img.getDepthImage(responses[1])
            
            depth_val = img_depth[(img_depth.shape[0]/2)-1][(img_depth.shape[1]/2)-1]
            msg_down  = img.CreateMonoMessage(img_down, sim_time, sequence)
            msg_alt   = img.CreateRangeMessage(depth_val, sim_time, sequence)
            msg_down_info   = img.CreateInfoMessage(img_down, FoV, sim_time, sequence)
            
            publisher_down.publish(msg_down)
            publisher_alt.publish(msg_alt)
            publisher_down_info.publish(msg_down_info) 
            
            if(LIDAR): 
                msg_depth = img.CreateDepthMessage(img_depth, sim_time, sequence)
                msg_depth_info  = img.CreateInfoMessage(img_depth, FoV, sim_time, sequence)
                publisher_depth.publish(msg_depth)
                publisher_depth_info.publish(msg_depth_info)
                
        if(STEREO): 
            responses = client.simGetImages([airsim.ImageRequest("2",airsim.ImageType.Scene,False,False),airsim.ImageRequest("1",airsim.ImageType.Scene,False,False)])
            
            img_left  = img.getRGBImage(responses[0])
            img_right = img.getRGBImage(responses[1])
            
            msg_left  = img.CreateMonoMessage(img_left, sim_time, sequence)
            msg_right  = img.CreateMonoMessage(img_right, sim_time, sequence)
            msg_left_info = img.CreateInfoMessage(img_left, FoV, sim_time, sequence)
            msg_right_info = img.CreateInfoMessage(img_right, FoV, sim_time, sequence)
            
            publisher_left.publish(msg_left) 
            publisher_right.publish(msg_right) 
            publisher_left_info.publish(msg_left_info) 
            publisher_right_info.publish(msg_right_info) 
            
    sequence += 1
subprocess.Popen("rosnode kill /cloudify /disparify /managerify /record_node /rectify /rosout", stdin=subprocess.PIPE, shell=True, cwd=path)
