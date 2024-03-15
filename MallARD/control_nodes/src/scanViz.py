#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import floor

def scan_viz_callback(scanData):
    scan_viz = scanData
    
    # delay = 0.5  # delay in seconds
    delay = rospy.get_param('~delay')
    
    # Combine the seconds and nano seconds into a float
    timeScanFloat = scanData.header.stamp.secs + (scanData.header.stamp.nsecs * (1e-9))
    # Subtract delay
    timeScanViz = timeScanFloat - delay

    # Divide the modified seconds and nano seconds back into two integers
    scan_viz.header.stamp.secs = int(floor(timeScanViz))
    scan_viz.header.stamp.nsecs = int((timeScanViz-scan_viz.header.stamp.secs) * (1e9))

    pub.publish(scan_viz)


def scan_viz():
    global pub

    rospy.Subscriber("/scan", LaserScan, scan_viz_callback)
    pub = rospy.Publisher("/scan_viz", LaserScan, queue_size=2)
    
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('scan_viz')
    scan_viz()
