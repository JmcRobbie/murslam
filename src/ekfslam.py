 #!/usr/bin/env python
 
import rospy 
'''
EKF slam implementation for MUR driverless

Author: Jack McRobbie 
'''
import numpy as np
import rospy 
import math 

class EkfSlam:
    
    def __init__(self,sensors):
        self.data = sensors 
    
    
    def doEKFSlam(self):
        '''
        Main ekfslam loop, will run until the node is terminated.
        '''
        
        while not rospy.is_shutdown():
            print "hello world" 