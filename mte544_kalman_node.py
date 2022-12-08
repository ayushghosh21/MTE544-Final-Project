#!/usr/bin/env python3

import numpy as np
from numpy.random import randn
import math
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

from nav_msgs.msg import Path

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('mte544_kalman_node')
        self.subscription_joint_state = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.subscription_joint_state  # prevent unused variable warning

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.path_pub = self.create_publisher(Path, '/path_viz', 10)


        self.subscription_imu = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            qos_profile=qos_policy)
        self.subscription_imu  # prevent unused variable warning
        
        self.rate = 10.0 # Hz
        self.dt = 1.0/self.rate
        
        self.timer = self.create_timer(self.dt, self.run_kalman_filter)

        # Initial estimate
        self.xhat = np.matrix([0.0, 0.0, 0.0]).transpose() # mean(mu) estimate for the "first" step
        self.P = 1 # covariance initial estimation between the
        self.path_list = [self.return_cart_pose(self.xhat)]

        self.joint_values = None
        
        self.omega = None
        self.acceleration = None
        self.accel_var = None        
        self.omega_var = None

        # robot wheel radius
        self.r = 0.066/2.0

        # Sensor measurement variance
        self.measurement_var = 4.3


    def joint_state_callback(self, msg):
        
        # [left wheel velocity, right wheel velocity]
        self.joint_values = msg.velocity

    def imu_callback(self, msg):
        self.omega = msg.angular_velocity.z

        self.acceleration = msg.linear_acceleration.x

        self.accel_var = msg.linear_acceleration_covariance[0]
        self.omega_var = msg.angular_velocity_covariance[0]

    def run_kalman_filter(self):
        
        if self.accel_var is None and self.omega_var is None:
            return
            
        self.A = np.matrix([
            [1.0, self.dt, 0.0], 
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        self.B = np.matrix([
            [0.5*(self.dt)**2, 0.0], 
            [self.dt, 0.0],
            [0.0, self.dt]
        ])

        self.Q = np.matrix([
            [(1.0/4.0*(self.dt)**4) * self.accel_var, (1.0/2.0*(self.dt)**3) * self.accel_var, 0.0],
            [(1.0/2.0*(self.dt)**3) * self.accel_var, ((self.dt)**2) * self.accel_var, 0.0],
            [0.0, 0.0, self.omega_var]
        ])
        
        self.C = np.matrix([
            [0.0, 1.0/self.r, 0.0],
            [0.0, 1.0/self.r, 0.0],
            [0.0, 0.0, 0.0],
        ])

         
        self.R = np.identity(3) * self.measurement_var
        


        # T = np.arange(0, Tfinal, self.dt)
        # xhat_S = np.zeros([3, len(T) + 1])
        # x_S = np.zeros([3, len(T) + 1])
        # x = np.zeros([3, len(T) + 1])
        # x[:, [0]] = self.xhat
        # y = np.zeros([2, len(T)])
        # y_hat = np.zeros([2, len(T)])



        u = np.matrix([self.acceleration, self.omega]).transpose()

        y = np.matrix([self.joint_values[0], self.joint_values[1], 0.0]).transpose() 
        # print(y.shape)
        
        #for k in range(len(T)):
            # u = 0.01 # normally you'd initialise this above

            # #### Simulate motion with random motion disturbance ####
            # w = np.matrix([self.Q[0, 0] * randn(1), self.Q[1, 1] * randn(1), self.Q[2, 2] * randn(1)])

            # # update state - this is a simulated motion and is PURELY for fake
            # # sensing and would essentially be
            # x[:, [k + 1]] = self.A * x[:, [k]] + self.B * u + w

            # # taking a measurement - simulating a sensor
            # # create our sensor disturbance
            # v = np.matrix([self.R[0, 0] * randn(1), self.R[1, 1] * randn(1)])
            # # create this simulated sensor measurement
            # y[:, [k]] = self.C*x[:, [k+1]] + v

        #########################################
        ###### Kalman Filter Estimation #########
        #########################################
        # Prediction update
        xhat_k = self.A * self.xhat + self.B * u # we do not put noise on our prediction
        P_predict = self.A*self.P*self.A.transpose() + self.Q
        # this co-variance is the prediction of essentially how the measurement and sensor model move together
        # in relation to each state and helps scale our kalman gain by giving
        # the ratio. By Definition, P is the variance of the state space, and
        # by applying it to the motion model we're getting a motion uncertainty
        # which can be propogated and applied to the measurement model and
        # expand its uncertainty as well

        # Measurement Update and Kalman Gain
        # K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)

        # print(P_predict)
        K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)

        
        # K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose())


        # the pseudo inverse of the measurement model, as it relates to the model covariance
        # if we don't have a measurement for velocity, the P-matrix tells the
        # measurement model how the two should move together (and is normalised
        # in the process with added noise), which is how the kalman gain is
        # created --> detailing "how" the error should be scaled based on the
        # covariance. If you expand P_predict out, it's clearly the
        # relationship and cross-projected relationships, of the states from a
        # measurement and motion model perspective, with a moving scalar to
        # help drive that relationship towards zero (P should stabilise).

        self.xhat = xhat_k + K * (y - self.C * xhat_k)
        self.P = (1 - K * self.C) * P_predict # the full derivation for this is kind of complex relying on
                                            # some pretty cool probability knowledge

        print(xhat_k)

        self.publish_path(xhat_k)
        # Store estimate
        
        # xhat_S[:, [k]] = xhat_k
        # x_S[:, [k]] = self.xhat
        # y_hat[:, [k]] = self.C*self.xhat


        # return x, xhat_S, x_S, y_hat

    def publish_path(self, xhat):
        #print(self.path_list)
        self.path_list.append(self.return_cart_pose(xhat))

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        for pose in  self.path_list:
            point = PoseStamped()

            #print(pose)
            point.pose.position.x = pose[0]
            point.pose.position.y = pose[1]

            path_msg.poses.append(point)

        self.path_pub.publish(path_msg)
    
    def return_cart_pose(self, xhat):
        x_pose = xhat[0] * np.cos(xhat.flat[2])
        y_pose = xhat[0] * np.sin(xhat.flat[2])
        return np.array([float(x_pose), float(y_pose)])

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()