#!/usr/bin/env python

"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import rospy

from sensor_msgs.msg import PointCloud, ChannelFloat32

from scipy.signal import find_peaks


class SonarFilter():
    def __init__(self):
        self.sonar_topic = "/tritech_micron/scan"
        #rospy.Timer(rospy.Duration(1.0), self.timedCB)
        self.minIntensity = 120.0
        self.minDistanceBetweenPeaks = 3.0 
        self.minDistancePoint = 1.0
        
        self.sub = rospy.Subscriber(self.sonar_topic, PointCloud, self.sonarCB, tcp_nodelay=True, queue_size=100)
        
        self.pubSonarMax = rospy.Publisher("/tritech_micron/scan/max", PointCloud, queue_size=10)
        self.pubSonarPeaks = rospy.Publisher("/tritech_micron/scan/peaks", PointCloud, queue_size=10)
    
    def sonarCB(self, msg):
        maxRange = np.linalg.norm(np.array([msg.points[-1].x, msg.points[-1].y, msg.points[-1].z]))
        distPerSample = maxRange/len(msg.points)
        #####MAX########
        ID = np.argmax(msg.channels[0].values)
        if np.linalg.norm(np.array([msg.points[ID].x, msg.points[ID].y, msg.points[ID].z])) >= self.minDistancePoint:
            PC = PointCloud()
            PC.header = msg.header
            PC.channels.append(ChannelFloat32("intensity", [msg.channels[0].values[ID]]))
            PC.points.append(msg.points[ID])
            self.pubSonarMax.publish(PC)
        
        
        ########PEAKS########## Needs minimum distance
        IDs, val = find_peaks(msg.channels[0].values, height=25, distance=int(self.minDistanceBetweenPeaks/distPerSample)) 
        PC = PointCloud()
        PC.header = msg.header
        print "-------------"
        print IDs
        print val
        PC.channels.append(ChannelFloat32("intensity", val['peak_heights']))
        for ID in IDs:
            PC.points.append(msg.points[ID])  
        self.pubSonarPeaks.publish(PC)
        
             

if __name__ == '__main__':
    try:
        rospy.init_node('SonarFilterNode', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    NODE = SonarFilter()
    rospy.spin()

