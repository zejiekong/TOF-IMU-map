#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nlink_parser.msg import TofsenseFrame0
from std_msgs.msg import Header
import math

class convertion:

    def __init__(self):
        self.pub = rospy.Publisher("scan",LaserScan,queue_size=1)
        rospy.Subscriber("nlink_tofsense_frame0",TofsenseFrame0,self.callback)
        rospy.spin()

    def callback(self,data):
        distance = data.dis 
        header = data.header
        laserdata = LaserScan()
        laserdata.header = header
        laserdata.angle_max = math.pi/2
        laserdata.angle_min = math.pi/2
        laserdata.time_increment = 0
        laserdata.angle_increment = 0
        laserdata.scan_time = 0
        laserdata.range_max = 50.0
        laserdata.range_min = 0
        laserdata.ranges = [distance,distance]
        self.pub.publish(laserdata)


    
if __name__ == "__main__":
    rospy.init_node("tof_to_laser",anonymous=True)
    convert = convertion()