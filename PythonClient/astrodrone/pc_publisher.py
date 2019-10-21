import setup_path, rospy, airsim, cv2, utils, numpy
from sensor_msgs.msg import PointCloud2, PointField

def CreatePC2Message(points, stamp, sequence):
	msg_pc = PointCloud2()
	msg_pc.header.stamp = stamp
	msg_pc.header.seq   = sequence 
	msg_pc.header.frame_id = "world" #"/down_camera_frame"

	if len(points.shape) == 3:
		msg_pc.height = points.shape[1]
		msg_pc.width = points.shape[0]
	else:
		msg_pc.height = 1
		msg_pc.width = len(points)

	msg_pc.fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1), PointField('z', 8, PointField.FLOAT32, 1)]
	msg_pc.is_bigendian = False
	msg_pc.point_step = 12
	msg_pc.row_step = 12*points.shape[0]
	msg_pc.is_dense = int(numpy.isfinite(points).all())
	msg_pc.data = numpy.asarray(points, numpy.float32).tostring() 
	return msg_pc







