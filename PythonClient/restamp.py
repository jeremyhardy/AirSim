import rosbag, sys, rospy

input_name = sys.argv[1]
output_name = sys.argv[2]

with rosbag.Bag(output_name, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_name).read_messages():
        if (msg._has_header):
        	    outbag.write(topic, msg, msg.header.stamp)
        elif (msg.transforms[0]._has_header): 
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
