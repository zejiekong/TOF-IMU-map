#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def callback(data):
    br = TransformBroadcaster()
    transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="map"), child_frame_id="imu")
    transform.transform.rotation = data.orientation
    br.sendTransformMessage(transform)
    
if __name__ == "__main__":
    rospy.init_node("imu_tf_broadcaster")
    rospy.Subscriber("imu/data",Imu,callback)
    rospy.spin()