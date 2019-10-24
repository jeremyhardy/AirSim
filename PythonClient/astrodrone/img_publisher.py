import math, cv2, airsim
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Range
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

def getDepthImage(response_d):
    img_depth = np.array(response_d.image_data_float, dtype=np.float32)
    img_depth = img_depth.reshape(response_d.height, response_d.width)
    return img_depth

def getRGBImage(response_rgb):
    img1d = np.fromstring(response_rgb.image_data_uint8, dtype=np.uint8)
    img_rgba = img1d.reshape(response_rgb.height, response_rgb.width, 3)
    img_rgb = img_rgba[..., :3][..., ::-1]
    return img_rgb
    
def CreateRGBMessage(img_rgb, img_time, sequence):
    msg_rgb = Image() 
    bridge_rgb = CvBridge() 
    msg_rgb.header.stamp = img_time
    msg_rgb.header.seq = sequence
    msg_rgb.header.frame_id = "world"
    msg_rgb.encoding = "bgr8"
    msg_rgb.height = img_rgb.shape[0]
    msg_rgb.width = img_rgb.shape[1]
    msg_rgb.data = bridge_rgb.cv2_to_imgmsg(img_rgb, "bgr8").data
    msg_rgb.is_bigendian = 0
    msg_rgb.step = msg_rgb.width * 3
    return msg_rgb

def CreateMonoMessage(img_rgb, img_time, sequence):
    msg_mono = Image() 
    bridge_mono = CvBridge() 
    img_mono = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    msg_mono.header.stamp = img_time
    msg_mono.header.seq = sequence
    msg_mono.header.frame_id = "world"
    msg_mono.encoding = "mono8"
    msg_mono.height = img_rgb.shape[0]
    msg_mono.width = img_rgb.shape[1] 
    msg_mono.data = bridge_mono.cv2_to_imgmsg(img_mono, "mono8").data
    msg_mono.is_bigendian = 0
    msg_mono.step = msg_mono.width
    return msg_mono

def CreateDepthMessage(img_depth, img_time, sequence):
    msg_d = Image()
    bridge_d = CvBridge()
    msg_d.header.stamp = img_time
    msg_d.header.seq = sequence
    msg_d.header.frame_id = "world"
    msg_d.encoding = "32FC1"
    msg_d.height = img_depth.shape[0]
    msg_d.width = img_depth.shape[1]
    msg_d.data = bridge_d.cv2_to_imgmsg(img_depth, "32FC1").data
    msg_d.is_bigendian = 0
    msg_d.step = msg_d.width * 4
    return msg_d

def CreateRangeMessage(rng, img_time, sequence):
    msg_range = Range() 
    msg_range.header.frame_id = "world"
    msg_range.header.seq = sequence
    msg_range.header.stamp = img_time
    msg_range.radiation_type = 1
    msg_range.field_of_view = 0
    msg_range.min_range = 0
    msg_range.max_range = 1000000
    msg_range.range = rng
    return msg_range

def CreateInfoMessage(img_in, fov, img_time, sequence):
    msg_info = CameraInfo()
    msg_info.header.frame_id = "world"
    msg_info.header.seq = sequence
    h = img_in.shape[0]
    w = img_in.shape[1]
    msg_info.height = h
    msg_info.width = w
    msg_info.distortion_model = "plumb_bob"
    
    CAMERA_K1 = 0.0
    CAMERA_K2 = 0.0
    CAMERA_P1 = 0.0
    CAMERA_P2 = 0.0
    CAMERA_P3 = 0.0
    
    fov = math.radians(fov / 2)
    CAMERA_FX = w / (2 * math.tan(fov))
    CAMERA_FY = h / (2 * math.tan(fov))
    CAMERA_CX = w / 2
    CAMERA_CY = h / 2
    
    msg_info.D.append(CAMERA_K1)
    msg_info.D.append(CAMERA_K2)
    msg_info.D.append(CAMERA_P1)
    msg_info.D.append(CAMERA_P2)
    msg_info.D.append(CAMERA_P3)
    
    msg_info.K[0] = CAMERA_FX
    msg_info.K[1] = 0
    msg_info.K[2] = CAMERA_CX
    msg_info.K[3] = 0
    msg_info.K[4] = CAMERA_FY
    msg_info.K[5] = CAMERA_CY
    msg_info.K[6] = 0
    msg_info.K[7] = 0
    msg_info.K[8] = 1
    
    msg_info.R[0] = 1
    msg_info.R[1] = 0
    msg_info.R[2] = 0
    msg_info.R[3] = 0
    msg_info.R[4] = 1
    msg_info.R[5] = 0
    msg_info.R[6] = 0
    msg_info.R[7] = 0
    msg_info.R[8] = 1
    
    msg_info.P[0] = CAMERA_FX
    msg_info.P[1] = 0
    msg_info.P[2] = CAMERA_CX
    msg_info.P[3] = 0
    msg_info.P[4] = 0
    msg_info.P[5] = CAMERA_FY
    msg_info.P[6] = CAMERA_CY
    msg_info.P[7] = 0
    msg_info.P[8] = 0
    msg_info.P[9] = 0
    msg_info.P[10] = 1
    msg_info.P[11] = 0
    
    msg_info.binning_x = msg_info.binning_y = 0
    msg_info.roi.x_offset = msg_info.roi.y_offset = msg_info.roi.height = msg_info.roi.width = 0
    msg_info.roi.do_rectify = False
    msg_info.header.stamp = img_time
    return msg_info
    
def CreateTFMessage(pose_msg, img_time, sequence):
    # down = airsim.Quaternionr(0, -0.7071067690849304, 0, 0.7071067094802856)
    down = airsim.Quaternionr(0, 0, 0, 1)
    enu  = airsim.Quaternionr(0.7071068, 0.7071068, 0, 0) 
    final = enu.__mul__(down) 
    msg_tf = TFMessage()
    msg_tf.transforms.append(TransformStamped())
    msg_tf.transforms[0].header.seq = sequence
    msg_tf.transforms[0].header.stamp = img_time
    msg_tf.transforms[0].header.frame_id = "/down_camera_frame"
    msg_tf.transforms[0].child_frame_id = "/world"
    msg_tf.transforms[0].transform.translation.x = pose_msg.position.x_val
    msg_tf.transforms[0].transform.translation.y = pose_msg.position.y_val
    msg_tf.transforms[0].transform.translation.z = pose_msg.position.z_val
    msg_tf.transforms[0].transform.rotation.x = final.x_val 
    msg_tf.transforms[0].transform.rotation.y = final.y_val
    msg_tf.transforms[0].transform.rotation.z = final.z_val
    msg_tf.transforms[0].transform.rotation.w = final.w_val
    return msg_tf
    
def CreateTFStampedMessage(pose_msg, img_time, sequence):
    msg_tf = TransformStamped()
    msg_tf.header.seq = sequence
    msg_tf.header.stamp = img_time
    msg_tf.header.frame_id = "/down_camera_frame"
    msg_tf.child_frame_id = "/world"
    msg_tf.transform.translation.x = pose_msg.position.x_val
    msg_tf.transform.translation.y = pose_msg.position.y_val
    msg_tf.transform.translation.z = pose_msg.position.z_val
    msg_tf.transform.rotation.x = -0.7071
    msg_tf.transform.rotation.y = 0.7071
    msg_tf.transform.rotation.z = 0
    msg_tf.transform.rotation.w = 0
    return msg_tf
