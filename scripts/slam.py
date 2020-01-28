#! usr/bin/env/python

'''
Python implementation of ekfslam for mapping of racecourse and for localizing a racecar. 

Author: Jack McRobbie
'''

import numpy as np 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
class slamTopics:
    def __init__(self):
        self.lidarCloudTopic = "/lidar/cones"
        self.cameraCloudTopic = "/camera/cones" 
class racecarSensors:
    '''
    A class that manages callbacks and sensors states for the racecar. 
    '''

    def __init__(self,topics):
        self.lidarTopic = topics.lidarCloudTopic
        self.cameraCloud = topics.cameraCloudTopic
        self.lidarCloud = np.array([]) 
        self.cameraCloud = np.array([])
    
    def lidarCallback(self, data):    
        dat = pc2.read_points(data,
                                field_names=("x", "y", "z"),
                                skip_nans=True)
        arr = list(dat)
        self.lidarCloud = np.array(arr)
        return 

    def cameraCallback(self,data):
        dat = pc2.read_points(data,
                        field_names=("x", "y", "z"),
                        skip_nans=True)
        arr = list(dat)
        self.cameraCloud = np.array(arr)
        return

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



def listener():
    topics =slamTopics() 

    sense = racecarSensors(topics)

    rospy.init_node('slam', anonymous=False)

    '''
    Call subscribers to various important nodes
    '''
    rospy.Subscriber(sense.lidarTopic, pc2, sense.lidarCallback)
    rospy.Subscriber(sense.cameraCallback, pc2, sense.cameraCloud)
    '''
    Slam begin!
    '''    
        
    rospy.spin()