#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Tritech Micron sonar scanner.

This publishes one PointCloud message per scan slice. In order to visualize in
rviz, play with the 'Decay Time' parameter. This node also provides parameters
that can be dynamically reconfigured.
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField, ChannelFloat32
from sensor_msgs import point_cloud2 as pc2
from tritech_micron import TritechMicron
from geometry_msgs.msg import PoseStamped
from tritech_micron.cfg import ScanConfig
from dynamic_reconfigure.server import Server
from tritech_micron.msg import TritechMicronConfig
import struct
__author__ = "Anass Al-Wohoush"


def reconfigure(config, level):
    """Reconfigures sonar dynamically.

    Args:
        config: New configuration.
        level: Level bitmask.

    Returns:
        Configuration.
    """
    rospy.loginfo("Reconfiguring sonar")
    rospy.logdebug("Configuration requested: %r, %r", config, level)

    # Remove additional keys.
    if "groups" in config:
        config.pop("groups")

    # Set parameters.
    sonar.set(**config)
    return config

def convertPointCloudToPointCloud2(inp):
    fields = []
    pfNames = ['x', 'y', 'z', 'intensity']
    pfOffset =[0,4,8,12]
    for i in range(3 + len(inp.channels)):
      fields.append(PointField(pfNames[i], pfOffset[i], PointField.FLOAT32, 1))
    data = []  
    for cp in range(len(inp.points)):
      data.append([inp.points[cp].x, inp.points[cp].y ,inp.points[cp].z, inp.channels[0].values[cp]])
    return pc2.create_cloud(inp.header, fields, data)

def publish(sonar, slice):
    """Publishes PointCloud, PoseStamped and TritechMicronConfig of current
    scan slice on callback.

    Args:
        sonar: Sonar instance.
        slice: Current scan slice.
    """

    # Publish heading as PoseStamped.
    posestamped = slice.to_posestamped(frame)
    heading_pub.publish(posestamped)

    # Publish data as PointCloud.
    cloud = slice.to_pointcloud(frame)
    scan_pub.publish(cloud)
 
    
    # Publish data as PointCloud2.
    cloud2 = convertPointCloudToPointCloud2(cloud)
    scan_pub2.publish(cloud2)

    # Publish data as TritechMicronConfig.
    config = slice.to_config(frame)
    conf_pub.publish(config)


if __name__ == "__main__":
    # Initialize node and publishers.
    rospy.init_node("tritech_micron")
    scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    maxInt_pub = rospy.Publisher("~MaxInt", PointCloud, queue_size=800)
    scan_pub2 = rospy.Publisher("~scan2", PointCloud2, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)
    conf_pub = rospy.Publisher("~config", TritechMicronConfig, queue_size=800)

    # Get frame name and port.
    frame = rospy.get_param("~frame")
    port = rospy.get_param("~port")

    with TritechMicron(port=port) as sonar:
        try:
            # Initialize dynamic reconfigure server and scan.
            Server(ScanConfig, reconfigure)

            # Scan.
            sonar.scan(callback=publish)
        except KeyboardInterrupt:
            sonar.preempt()
