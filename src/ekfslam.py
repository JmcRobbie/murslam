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
        self.minDistThresh = 1  # Minumum distance threshold

        # EKF state variance
        self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2
        self.Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
        self.R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2

    def doEKFSlam(self):
        '''
        Main ekfslam loop, will run until the node is terminated.
        '''
        rospy.loginfo("Doing ekf slam at %.0f HZ \n", self.rate)

        while not rospy.is_shutdown():
            rate = rospy.Rate(self.rate)
            rate.sleep()

    def ekfSlam(self, X_est, Pest, cam_obs, lidar_obs):
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
        # Update ignoring camera based observations for now
        for coneOb in range(len(lidar_obs)):
            min_id = self.search_correspond_landmark_id(
                X_est, Pest, coneOb[0:2])
            num_cones = self.calcNumCones(X_est)
            if min_id == num_cones:
                ros.loginfo(
                    "New cone identified \nExtending covariance matrix")
                Xtmp = np.vstack(
                    (X_est, cam_obs))
                Ptmp = np.vstack((np.hstack((Pest, np.zeros((len(X_est), 2)))),
                                  np.hstack((np.zeros((2, len(X_est))), initP))))
                X_est = Xtmp
                Pest = Ptmp
            lm = get_landmark_position_from_state(X_est, min_id)
            y, s, H = calc_innovation(lm, X_est, Pest, coneOb, min_id)
            K = np.matmul(Pest, H.T, np.linalg.inv(s))
            X_est = X_est + np.matmul(K, y)
            Pest = np.eye(len(X_est)) - np.matmul(K, H, Pest)

        X_est[2] = pi_2_pi(X_est[2])
        return X_est, Pest

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

    def search_correspond_landmark_id(self, X_est, P_est, landmark):
        """
        Landmark association with Mahalanobis distance
        """

        nLM = self.calcNumCones(X_est)

        min_dist = []

        for i in range(nLM):
            lm = self.get_landmark_position_from_state(X_est, i)
            y, S, H = self.calc_innovation(lm, X_est, P_est, landmark, i)
            min_dist.append(np.matmul(y.T, np.linalg.inv(S), y))

        min_dist.append(self.minDistThresh)  # new landmark

        min_id = min_dist.index(min(min_dist))

        return min_id
