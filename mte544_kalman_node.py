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
        
        self.rate = 5.0 # Hz
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

        # robot wheel base distance
        self.F = 0.08*2.0
        
        # Sensor measurement variance
        self.measurement_var = 4.3


    def joint_state_callback(self, msg):
        
        # [left wheel velocity, right wheel velocity]
        self.joint_values = msg.velocity
        #print("vel", self.joint_values[0], self.joint_values[1])

    def imu_callback(self, msg):
        self.omega = msg.angular_velocity.z

        self.acceleration = msg.linear_acceleration.x

        self.accel_var = msg.linear_acceleration_covariance[0]
        self.omega_var = msg.angular_velocity_covariance[0]

        #print(self.omega, self.acceleration, self.accel_var, self.omega_var)

    def run_kalman_filter(self):
        if self.accel_var is None and self.omega_var is None:
            return
            
        self.A = np.matrix([
            [1.0, self.dt, 0.0, 0.0], 
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        
        self.B = np.matrix([
            [0.5*(self.dt)**2, 0.0], 
            [self.dt, 0.0],
            [0.0, self.dt],
            [0.0, 1.0]
        ])

        self.Q = np.matrix([
            [(1.0/4.0*(self.dt)**4) * self.accel_var, (1.0/2.0*(self.dt)**3) * self.accel_var, 0.0, 0.0],
            [(1.0/2.0*(self.dt)**3) * self.accel_var, ((self.dt)**2) * self.accel_var, 0.0, 0.0],
            [0.0, 0.0, ((self.dt)**2) * self.omega_var, self.dt * self.omega_var],
            [0.0, 0.0, self.dt * self.omega_var, self.omega_var],
        ])
        
        self.C = np.matrix([
            [0.0, 1.0/self.r, 0.0, self.F/2.0],
            [0.0, 1.0/self.r, 0.0, -self.F/2.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        
        self.R = np.identity(3) * self.measurement_var
        

        u = np.matrix([self.acceleration, self.omega]).transpose()
        

        self.omega_soft = self.joint_values[0] * self.joint_values[1]
        y = np.matrix([self.joint_values[0], self.joint_values[1], self.omega_soft]).transpose() 

        #########################################
        ###### Kalman Filter Estimation #########
        #########################################
        # Prediction update
        xhat_k = self.A * self.xhat + self.B * u # we do not put noise on our prediction
        P_predict = self.A*self.P*self.A.transpose() + self.Q
        
        K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)

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