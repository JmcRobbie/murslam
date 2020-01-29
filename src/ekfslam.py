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

    def __init__(self, carSensors):

        self.sensors = carSensors
        self.rate = 10
        self.dt = 1.0/self.rate
        self.stateSize = 3  # [x y yaw]
        self.coneStateSize = 2

        # EKF state variance
        self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2
        self.Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
        self.R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2

    def doEKFSlam(self):
        '''
        Main ekfslam loop, will run until the node is terminated.
        '''

        while not rospy.is_shutdown():
            rate = rospy.Rate(self.rate)
            rospy.loginfo("Doing ekf slam at 10 HZ \n")
            rate.sleep()

    def ekfSlam(self, X_est, Pest):
        # Predict
        u = self.sensors.driveCmd
        X_est = self.motionModel(
            X_est[0:self.stateSize],
            u
        )
        G, Fx = self.jacob_motion(X_est[0:self.stateSize],
                                  u)
        Pest[0:self.stateSize, 0:self.stateSize] = np.matmul(G.T,
                                                             Pest[0:self.stateSize,
                                                                  0:self.stateSize],
                                                             G) + np.matmul(Fx.T, self.Cx, Fx)
        # Update
        for coneOb in range(2):
            num_cones = self.calcNumCones(X_est)

    def motionModel(self, x, u):
        F = np.array([[1.0, 0, 0],
                      [0, 1.0, 0],
                      [0, 0, 1.0]])

        B = np.array([[self.dt * math.cos(x[2, 0]), 0],
                      [self.dt * math.sin(x[2, 0]), 0],
                      [0.0, self.dt]])
        x = np.matmul(F, x) + np.matmul(B, u)
        return x

    def jacob_motion(self, x, u):

        Fx = np.hstack((np.eye(self.stateSize), np.zeros(
            (self.stateSize, self.coneStateSize * self.calcNumCones(x)))))

        jF = np.array([[0.0, 0.0, -self.dt * u[0] * math.sin(x[2, 0])],
                       [0.0, 0.0, self.dt * u[0] * math.cos(x[2, 0])],
                       [0.0, 0.0, 0.0]])

        G = np.eye(self.stateSize) + np.matmul(Fx.T, jF, Fx)

        return G, Fx,

    def calcNumCones(self, x):
        n = int((len(x) - self.stateSize) / self.coneStateSize)
        return n
