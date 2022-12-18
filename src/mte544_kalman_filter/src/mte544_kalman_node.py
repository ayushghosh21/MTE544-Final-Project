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
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('mte544_kalman_node')

        # Define kalman filter rate and corresponding delta_T 
        self.rate = 25 # Hz
        self.dt = 1.0/self.rate

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        # Define /joint_states topic subscriber. Values used in sensor model
        self.subscription_joint_state = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            qos_profile=qos_policy)
        self.subscription_joint_state  # prevent unused variable warning
        
        # Define /imu topic subscriber. Values used in motion model
        self.subscription_imu = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            qos_profile=qos_policy)
        self.subscription_imu  # prevent unused variable warning
        
        # Define /odom topic subscriber. Used for collecting ground truth
        self.subscription_tf = self.create_subscription(
            TFMessage,
            'tf',
            self.tf_callback,
            qos_profile=qos_policy)
        self.subscription_tf  # prevent unused variable warning
        
        # Define path publisher. Used to visualize estimated path in rviz
        self.path_pub = self.create_publisher(Path, '/path_viz', 10)
        
        # Define ground truth path publisher. Used to visualize ground truth path in rviz
        self.viz_pub = self.create_publisher(MarkerArray, 'ground_truth_viz', 10)

        # Store current Ground truth global poses as [x, y]
        self.curr_ground_truth_pose = [0.0, 0.0]

        # Store ground truth list
        self.ground_truth_list = []
        self.ground_truth_list.append([0.0, 0.0]) # Appending initial robot pose

        

        # Initial estimates at k-1
        self.xhat = np.matrix([0.0, 0.0, 0.0, 0.0]).transpose() # "predicted" system state vector at k-1

        #prior estimate covariance matrix of the current state "predicted" at k-1
        self.P = np.identity(4)
        
        # Storing a list of points travelled by robot
        self.path_list = []
        self.path_list.append([0.0, 0.0]) # Appending initial robot pose

        # Storing distance estimate, used by path publisher later
        self.previous_dist_value = 0.0
        
        # Declaring wheel speed array
        self.joint_values = None # Will have structure [left_wheel_speed, right_wheel_speed]
        
        # Declaring control variables. Will be populated by /imu topic callback 
        self.omega = None
        self.acceleration = None

        # Declaring acceleration and angular velocity variance variables. 
        # Will be populated by /imu topic callback
        self.accel_var = None        
        self.omega_var = None

        # Defining robot wheel radius (r) from diameter (D)
        D = 0.066
        self.r = D/2.0

        # robot wheel base distance
        self.F = 0.16
        
        # Sensor measurement variance
        self.measurement_var = 0.1
        
        
        # First time flag. This is used to define the Kalman matricies one time, 
        # after the variance values are read from the imu_topic
        self.first_time = True


        # Initializing timer to run kalman filter function at specified rate
        self.timer = self.create_timer(self.dt, self.run_kalman_filter)

        self.heartbeat = False
        
    def joint_state_callback(self, msg):
        """Storing joint velocities from the /joint_states topic"""

        # [left wheel velocity, right wheel velocity]
        self.joint_values = msg.velocity


    def imu_callback(self, msg):
        """Storing the angular velocity, linear acceleration, as well as their attributed variances from the /imu topic"""

        self.omega = msg.angular_velocity.z

        self.acceleration = msg.linear_acceleration.x

        self.accel_var = msg.linear_acceleration_covariance[0]
        self.omega_var = msg.angular_velocity_covariance[0]
        
        # Signal that still receiving messages
        self.heartbeat = True
    
    def tf_callback(self, msg):
        """Storing the global pose from the /tf topic"""
        all_tfs = msg.transforms
        
        for tf in all_tfs:
            if tf.child_frame_id == "base_footprint":
                self.curr_ground_truth_pose = [tf.transform.translation.x, tf.transform.translation.y]

    def run_kalman_filter(self):
        
        # if imu topic is being published then this heartbeat check will always pass
        # Used as a way of stopping the node from running iterations of 
        # kalman filter on the same data when ros2 bag or imu topic stopped.
        if self.heartbeat:
            self.heartbeat = False
        else:
            return

        # Stall timer function if no values from imu or joint_states or tf have been 
        # received by the topic callback functions
        if self.accel_var is None and self.omega_var is None and self.joint_values is None and self.curr_ground_truth_pose is None:
            return

        if self.first_time:
            # Now that variance values are read in, we can calculate the kalman matrices one time.

            # Kalman Filter Matrices defined below:
        
            # State Space matrix
            self.A = np.matrix([
                [1.0, self.dt, 0.0, 0.0], 
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0]
            ])
            
            # Control matrix
            self.B = np.matrix([
                [0.5*(self.dt)**2, 0.0], 
                [self.dt, 0.0],
                [0.0, self.dt],
                [0.0, 1.0]
            ])

            # Process noise matrix
            self.Q = np.matrix([
                [(1.0/4.0*(self.dt)**4) * self.accel_var, (1.0/2.0*(self.dt)**3) * self.accel_var, 0.0, 0.0],
                [(1.0/2.0*(self.dt)**3) * self.accel_var, (self.dt**2) * self.accel_var, 0.0, 0.0],
                [0.0, 0.0, ((self.dt)**2) * self.omega_var, self.dt * self.omega_var],
                [0.0, 0.0, self.dt * self.omega_var, self.omega_var],
            ])
            
            # Observation matrix
            self.C = np.matrix([
                [0.0, 1.0/self.r, 0.0, -self.F/(self.r*2.0)],
                [0.0, 1.0/self.r, 0.0, self.F/(self.r*2.0)],
                [0.0, 0.0, 0.0, 1.0],
            ])

            # Measurement Uncertainty
            self.R = np.identity(3) * self.measurement_var

            self.first_time = False

        #########################################
        ###### Kalman Filter Estimation #########
        #########################################

        # control vector containing [lin_accel in X, ang_vel in Z]
        u = np.matrix([self.acceleration, self.omega]).transpose()
        
        left_wheel_encoder_speed = self.joint_values[0]
        right_wheel_encoder_speed = self.joint_values[1]
        
        # calculating the "soft" sensor omega using kinematic equations and wheel joint speeds 
        self.omega_soft = self.r * (right_wheel_encoder_speed - left_wheel_encoder_speed)/self.F
        
        # Defining the measurement vector
        y = np.matrix([left_wheel_encoder_speed, right_wheel_encoder_speed, self.omega_soft]).transpose() 

        # Prediction
        xhat_k = self.A * self.xhat + self.B * u
        
        P_predict = self.A*self.P*self.A.transpose() + self.Q
        
        # Update
        K = P_predict * self.C.transpose()*np.linalg.inv(self.C*P_predict*self.C.transpose() + self.R)

        self.xhat = xhat_k + K * (y - self.C * xhat_k)
        
        # Using full derivation when updating P to avoid matrix singularities
        self.P = (np.identity(4)-K*self.C)*P_predict * np.matrix((np.identity(4)-K*self.C)).transpose() + K*self.R*K.transpose()
        
        # Define the delta distance and theta value to pass into the publish function
        delta_dist = xhat_k.flat[0] - self.previous_dist_value
        theta = xhat_k.flat[2]
        self.publish_path(delta_dist, theta)
        self.previous_dist_value = xhat_k.flat[0]

        self.ground_truth_list.append(self.curr_ground_truth_pose)        
        self.visualize_ground_truth(self.ground_truth_list)

        mse = self.calculate_MSE()

        self.get_logger().info(f"MSE of estimation: {mse}")

    def publish_path(self, delta_x, theta):

        self.path_list.append(self.return_cart_pose(delta_x, theta))
        

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        for pose in  self.path_list:
            point = PoseStamped()

            point.pose.position.x = pose[0]
            point.pose.position.y = pose[1]

            path_msg.poses.append(point)

        self.path_pub.publish(path_msg)


    def return_cart_pose(self, delta_dist, theta):
        prev_pose = self.path_list[-1]
        
        delta_x_pose = delta_dist * np.cos(theta)
        delta_y_pose = delta_dist * np.sin(theta)
        
         
        return np.array([float(prev_pose[0] + delta_x_pose), float(prev_pose[1] + delta_y_pose)])

    def calculate_MSE(self):
        """calculate MSE using the ground truth pose list and estimated path pose list"""
        
        sum_square_error = 0
        n = len(self.ground_truth_list)
        for gt_pose, est_pose in zip(self.ground_truth_list, self.path_list):
            # Finding eucledian distance between ground truth and estimated point
            square_error = math.sqrt((gt_pose[0] - est_pose[0])**2 + (gt_pose[1] - est_pose[1])**2)
            sum_square_error += square_error
        
        mse = sum_square_error/n
        
        return mse
    
    def visualize_ground_truth(self, points):
        
        markers = MarkerArray()
        
        for idx, val in enumerate(points):
            ## To produce dotted line effect, only include every 15th point
            if idx%15 == 0:
                if val is None:
                    return
                ellipse = Marker()
                ellipse.header.frame_id = 'odom'
                ellipse.header.stamp = self.get_clock().now().to_msg()
                ellipse.pose.position.x = val[0] 
                ellipse.pose.position.y = val[1]
                ellipse.pose.position.z = 0.0
                ellipse.pose.orientation.y = 0.707
                ellipse.pose.orientation.w = 0.707
                
                ellipse._id = idx

                ellipse.type = Marker.CYLINDER
                ellipse.scale.x = 0.02
                ellipse.scale.y = 0.02
                ellipse.scale.z = 0.02
                ellipse.pose.position.z = 0.0
                
                ellipse.color.a = 1.0
                ellipse.color.r = 1.0
                ellipse.color.g = 0.0
                ellipse.color.b = 0.0
                
                markers.markers.append(ellipse)

        self.viz_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()