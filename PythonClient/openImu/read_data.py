import setup_path, airsim, utils, math
import pandas as pd

def readData(foldername):
    position = pd.read_csv(foldername+'ref_pos.csv')
    attitude = pd.read_csv(foldername+'ref_att_quat.csv')
    accelerometer = pd.read_csv(foldername+'ref_accel.csv')
    gyroscope = pd.read_csv(foldername+'ref_gyro.csv')
    time = pd.read_csv(foldername+'time.csv')
    noisy_gyro = pd.read_csv(foldername+'gyro-0.csv')
    noisy_accel = pd.read_csv(foldername+'accel-0.csv')
    return position, attitude, accelerometer, gyroscope, noisy_accel, noisy_gyro, time

def getPosition(df):
    x = df['ref_pos_x (m)']
    y = df['ref_pos_y (m)']
    z = df['ref_pos_z (m)']

    position_new = [] 
    position_start = airsim.Vector3r(x[0], y[0], z[0])
    for i in range(len(z)):
        # the raw values from openIMU are ecef_initial + relative position (subtract out any originial value for true relative)
        relative_position = airsim.Vector3r(x[i], y[i], z[i]).__sub__(position_start) 
        position_new.append(relative_position)
            
    return position_new
    
def getAttitude(df):
    qw = df['q0 ()']
    qx = df['q1']
    qy = df['q2']
    qz = df['q3']

    quaternion_new = []

    for i in range(len(qz)):
        quaternion_new.append(airsim.Quaternionr(qx[i], qy[i], qz[i], qw[i])) # no indication if this is a left or right quaternion  
            
    return quaternion_new
    
def getTime(df):
    t = df['time (sec)']
    
    time_new = []
    
    for i in range(len(t)):
        time_new.append(t[i])  # append time 
            
    return time_new
        
def getAccel(df): 
    
    ax = df['ref_accel_x (m/s^2)']
    ay = df['ref_accel_y (m/s^2)']
    az = df['ref_accel_z (m/s^2)']
    
    accel_data = [] 
    
    for i in range(len(az)): 
        accel_data.append(airsim.Quaternionr(ax[i], ay[i], az[i], 0))
                  
    return accel_data
    
def getGyro(df): 

    x_dot = df['ref_gyro_x (deg/s)']
    y_dot = df['ref_gyro_y (deg/s)']
    z_dot = df['ref_gyro_z (deg/s)']
    
    gyro_data = []
    deg2rad = math.pi/180
    for i in range(len(z_dot)): 
        gyro_data.append(airsim.Quaternionr(x_dot[i]*deg2rad, y_dot[i]*deg2rad, z_dot[i]*deg2rad, 0))       
                  
    return gyro_data    
    
def getNoisyAccel(df): 
    
    ax = df['accel_x (m/s^2)']
    ay = df['accel_y (m/s^2)']
    az = df['accel_z (m/s^2)']
    
    accel_data = [] 
    
    for i in range(len(az)): 
        accel_data.append(airsim.Quaternionr(ax[i], ay[i], az[i], 0))
                  
    return accel_data
    
def getNoisyGyro(df): 

    x_dot = df['gyro_x (deg/s)']
    y_dot = df['gyro_y (deg/s)']
    z_dot = df['gyro_z (deg/s)']
    
    gyro_data = []
    deg2rad = math.pi/180
    for i in range(len(z_dot)): 
        gyro_data.append(airsim.Quaternionr(x_dot[i]*deg2rad, y_dot[i]*deg2rad, z_dot[i]*deg2rad, 0))       
                  
    return gyro_data       
