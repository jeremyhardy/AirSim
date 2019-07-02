import sys, pdb
sys.path.insert(0, '../ros/')

import setup_path, airsim, os, rospy, cv2, math, subprocess, utils, math 
import pandas as pd
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Range, Imu
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
import img_publisher as img
import imu_publisher as inertial
import read_data as traj
from collections import OrderedDict

inname = "/home/tigerteam/landerNEDTrajectory_with_IMUBodyData-2.xlsx"
outname = "/home/tigerteam/gnss-ins-sim/demo_motion_def_files/motion_def-python.csv" 

data = pd.read_excel(inname, sheet="log_truth")
rad2deg = 180/math.pi

ax = data['lander_sensors.imu.output.dLinearAccel[0] {m/s2}'] 
ay = data['lander_sensors.imu.output.dLinearAccel[1] {m/s2}']
az = data['lander_sensors.imu.output.dLinearAccel[2] {m/s2}']

gx = data['lander_sensors.imu.output.dAngularVel[0] {rad/s}']*rad2deg
gy = data['lander_sensors.imu.output.dAngularVel[1] {rad/s}']*rad2deg
gz = data['lander_sensors.imu.output.dAngularVel[2] {rad/s}']*rad2deg

vx = data['lander_aux.landerLSiteNED.trans.velocity[0] {m/s}'] 
vy = data['lander_aux.landerLSiteNED.trans.velocity[1] {m/s}'] 
vz = data['lander_aux.landerLSiteNED.trans.velocity[2] {m/s}'] 

command = []
duration = []  
gps = []
gx = [] 
gy = [] 
gz = [] 
aax = [] 
aay = [] 
aaz = [] 

for i in range(len(ax)-1): 
    command.append(1) 
    gps.append(0) 
    duration.append(0.05)
    gx.append(0) 
    gy.append(0) 
    gz.append(0) 
    aax.append((vx[i+1]-vx[i])/0.05) 
    aay.append((vy[i+1]-vy[i])/0.05) 
    aaz.append(-(vz[i+1]-vz[i])/0.05) # random negative because gravity doesn't exist and god is a lie. 
    
header_columns = ['ini lat (deg)','ini lon (deg)','ini alt (m)','ini vx body (m/s)','ini vy body (m/s)','ini vz body (m/s)','ini yaw (deg)','ini pitch (deg)','ini roll (deg)']
data_columns = ['command type', 'yaw (deg)', 'pitch (deg)', 'roll (deg)', 'vx_body (m/s)', 'vy_body (m/s)', 'vz_body (m/s)', 'command duration (s)', 'GPS visibility']

df1 = pd.DataFrame( {'ini lat (deg)':[0], 'ini lon (deg)':[1], 'ini alt (m)':[2], 'ini vx body (m/s)':[3], 'ini vy body (m/s)':[4], 'ini vz body (m/s)':[5], 'ini yaw (deg)':[6], 'ini pitch (deg)':[7], 'ini roll (deg)':[8] }, columns=header_columns )

df2 = pd.DataFrame( {'command type':pd.Series(command).dropna(),  'yaw (deg)': pd.Series(gx).dropna(), 'pitch (deg)': pd.Series(gz).dropna(), 'roll (deg)': pd.Series(gy).dropna(), 'vx_body (m/s)': pd.Series(aax).dropna(), 'vy_body (m/s)': pd.Series(aay).dropna(), 'vz_body (m/s)': pd.Series(aaz).dropna(), 'command duration (s)': pd.Series(duration).dropna(), 'GPS visibility': pd.Series(gps).dropna()}, columns=data_columns )

df2.to_csv(outname, encoding='utf-8', index=False) 
