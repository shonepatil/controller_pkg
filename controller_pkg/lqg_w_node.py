import os
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from .controller_submodule.lqr_calculator import LQRDesign
from .controller_submodule.car_model import CarModel
from .controller_submodule.linear_kalman_filter import LinearKalmanFilter
import numpy as np

NODE_NAME = 'lqg_node'
ACTUATOR_TOPIC_NAME = '/cmd_vel'

POSE_TOPIC_NAME = '/amcl_pose'
PATH_TOPIC_NAME = '/global_trajectory'
ERROR_TOPIC_NAME = '/path_error'
IMU_TOPIC_NAME = '/imu'
ODOM_TOPIC_NAME = '/odom'


class LqgController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        ### Get sensor measurements ###
        #
        # Get IMU measurement
        self.velocity_subscriber = self.create_subscription(Imu, IMU_TOPIC_NAME, self.imu_measurement, 10)
        self.velocity_subscriber

        # Get GPS/Lidar measurements
        self.pose_subscriber = self.create_subscription(Pose, POSE_TOPIC_NAME, self.pose_measurement, 10)
        self.pose_subscriber

        # Get Odometry measurements
        self.odom_subscriber = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odom_measurement, 10)
        self.odom_subscriber

        # Get Reference Trajectory
        self.path_subscriber = self.create_subscription(Path, PATH_TOPIC_NAME, self.set_path, 10)
        self.path_subscriber

        # TODO: Make compatible with Camera Navigation via: Lane detection
        # Get road marker error measurements from camera
        # self.pose_error_subscriber = self.create_subscription(Float32MultiArray, ERROR_TOPIC_NAME, self.camera_measurment, 10)
        # self.pose_error_subscriber

        ### Controller and State Estimate modules ###
        self.car_model = CarModel()
        self.lqr_calc = LQRDesign(self.car_model)
        self.kalman_calc = LinearKalmanFilter()
        self.P = np.diag([1, 1, 1, 1])
        self.Qo = np.diag([1, 1, 1, 1])
        self.Ro = [0.1]
        self.x0 = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.state_measurement = self.x0
        self.state_est = self.x0

        # Filtered states
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.yaw_rate = 0
        self.vx = 0
        self.vy = 0

        # Sensor measurements Buffer
        # Lidar
        self.x_lidar_buffer = 0
        self.y_lidar_buffer = 0
        self.yaw_lidar_buffer = 0

        # IMU
        self.yaw_imu_buffer = 0
        self.yaw_rate_imu_buffer = 0

        # VESC
        self.x_vesc_buffer = 0
        self.y_vesc_buffer = 0
        self.yaw_vesc_buffer = 0
        self.yaw_rate_vesc_buffer = 0
        self.vx_vesc_buffer = 0.1
        self.vy_vesc_buffer = 0

        # JOY
        self.joy_speed_buffer = 0
        self.joy_steering_buffer = 0

        # Path coordinates
        self.x_path = []
        self.y_path = []
        self.yaw_path = []

        # Calculated states
        self.ecg = 0  # cross-track error
        self.theta_e = 0  # heading error
        self.theta_e_dot = 0  # heading error rate

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('error_threshold', 0.15),
                ('zero_speed', 0.0),
                ('max_speed', 5),
                ('min_speed', 0.1),
                ('max_right_steering', 0.4),
                ('max_left_steering', -0.4)
            ])
        self.error_threshold = self.get_parameter('error_threshold').value  # between [0,1]
        self.zero_speed=self.get_parameter('zero_speed').value  # should be around 0
        self.max_speed=self.get_parameter('max_speed').value  # between [0,5] m/s
        self.min_speed=self.get_parameter('min_speed').value  # between [0,5] m/s 
        self.max_right_steering=self.get_parameter('max_right_steering').value  # negative(max_left) 
        self.max_left_steering=self.get_parameter('max_left_steering').value  # between abs([0,0.436332]) radians (0-25degrees)

        self.get_logger().info(
            f'\nerror_threshold: {self.error_threshold}'
            f'\nzero_speed: {self.zero_speed}'
            f'\nmax_speed: {self.max_speed}'
            f'\nmin_speed: {self.min_speed}'
            f'\nmax_right_steering: {self.max_right_steering}'
            f'\nmax_left_steering: {self.max_left_steering}'
            f'\nself.state_measurement: {self.state_measurement}'
            f'\ntype(self.state_measurement): {type(self.state_measurement)}'
            f'\nshape(self.state_measurement): {self.state_measurement.shape}'
        )

        # Call controller
        self.Ts = 1/100  # contoller publish frequency (Hz)
        self.create_timer(self.Ts, self.controller)

    def imu_measurement(self, imu_data):
        # self.get_logger().info("Updating IMU")
        quaternion = (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)
        euler = euler_from_quaternion(quaternion)
        
        self.yaw_imu_buffer = euler[2]
        self.yaw_rate_imu_buffer = imu_data.angular_velocity.z

    def odom_measurement(self, odom_data):
        # car position
        self.x_vesc_buffer = odom_data.pose.pose.position.x
        self.y_vesc_buffer = odom_data.pose.pose.position.y

        # car linear velocity
        self.vx_vesc_buffer = odom_data.twist.twist.linear.x
        self.vy_vesc_buffer = odom_data.twist.twist.linear.y

        # car orientation
        quaternion = (odom_data.orientation.x, odom_data.orientation.y, odom_data.orientation.z, odom_data.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw_vesc_buffer = euler[2]

        # car angular velocity
        self.yaw_rate_vesc_buffer = odom_data.twist.twist.angular.z

    def pose_measurement(self, pose_data):
        # car orientation
        quaternion = (pose_data.pose.orientation.x, pose_data.pose.orientation.y, pose_data.pose.orientation.z, pose_data.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw_lidar_buffer = euler[2]

        # car position
        self.x_lidar_buffer = pose_data.pose.position.x
        self.y_lidar_buffer = pose_data.pose.position.y
        # self.get_logger().info(f"Updating POSE (x): ({self.x})")

    def set_path(self, path_data):
        # path coordinates (GLOBAL)
        self.x_path = np.array([pose.pose.position.x for pose in path_data.poses])
        self.y_path = np.array([pose.pose.position.x for pose in path_data.poses])
        # self.get_logger().info(f"first val PATH (x): ({self.x_path[0]})")

    def set_joy_command(self, joy_data):
        self.joy_speed_buffer = joy_data.drive.speed
        self.joy_steering_buffer = joy_data.drive.steering_angle

    def get_cross_track_error(self):
        # self.get_logger().info("Updating CROSS-TRACK-ERROR")
        
        # find 2 closest points in path with car
        error_x = self.x_path - self.x
        error_y = self.y_path - self.y
        error_mag = np.power(np.power(error_x,2) + np.power(error_y, 2), 0.5)
        error_mag1, error_mag2 = np.partition(error_mag, 1)[0:2]
        error_mag1_index = np.argwhere(error_mag == error_mag1)[0][0]
        error_mag2_index = np.argwhere(error_mag == error_mag2)[0][0]
        Px1 = self.x_path[error_mag1_index]
        Px2 = self.x_path[error_mag2_index]
        Py1 = self.y_path[error_mag1_index]
        Py2 = self.y_path[error_mag2_index]
        
        # create line extrapolations to determine cross-track error
        # (threshold added to account for zero/infinite slopes)
        
        # path line
        delta_x = Px2 - Px1 + self.line_error_threshold
        delta_y = Py2 - Py1 + self.line_error_threshold
        theta_path = float(np.arctan2(delta_y, delta_x))
        path_slope = delta_y / delta_x
        path_intercept = Py1 - path_slope * Px1
        
        # car line
        car_slope = -1 / path_slope
        car_intercept = self.y - car_slope * self.x
        ecg_x = (path_intercept - car_intercept) / (car_slope - path_slope)
        ecg_y = car_slope * ecg_x + car_intercept

        # get actual cross-track error distance and use sign of slope to determine direction
        ecg_r = np.power(np.power((self.x - ecg_x),2) + np.power((self.y - ecg_y), 2), 0.5)
        e_cg_sign = np.sign(car_slope)
        e_cg = float(e_cg_sign * ecg_r)
        self.get_logger().info(f"{e_cg},{theta_path}")
        return e_cg, theta_path

    def get_latest_measurements(self):
        # car orientation
        self.yaw = float(np.mean([self.yaw_lidar_buffer, self.yaw_imu_buffer, self.yaw_vesc_buffer]))

        # car angular speed
        self.yaw_rate = float(np.mean([self.yaw_rate_imu_buffer, self.yaw_rate_vesc_buffer]))
        
        # car coordinates
        self.x = float(np.mean([self.x_lidar_buffer, self.x_vesc_buffer]))
        self.y = float(np.mean([self.y_lidar_buffer, self.y_vesc_buffer]))

        # car linear speed
        self.vx = self.vx_vesc_buffer
        self.vy = self.vy_vesc_buffer

        # manual control
        self.joy_speed = self.joy_speed_buffer 
        self.joy_steering = self.joy_steering_buffer

        # time
        self.current_time = self.get_clock().now().to_msg()


    def update_gains(self):
        K = self.lqr_calc.compute_single_gain_sample(self.sys)
        return K

    def update_states(self):
        # self.get_logger().info("Updating STATES")
        theta_e_km1 = self.state_measurement[2][0]
        e_cg, theta_path = self.get_cross_track_error()
        theta_e_k = theta_path - self.yaw
        self.state_measurement[0][0] = e_cg
        self.state_measurement[1][0] = self.vy + self.vx * math.sin(theta_e_k)
        self.state_measurement[2][0] = theta_e_k
        self.state_measurement[3][0] = (theta_e_k - theta_e_km1) / self.Ts
        self.get_logger().info(f"states: {self.state_measurement}")

    def controller(self):
        """
        Need:
        -pose data
        -path data
        -vx (measured longitudinal velocity)

        states:
        -ecg (cross-trackk error from center of gravity (cg))
        -ecg_dot (cross-trackk error rate from cg)
        -theta_e (heading error)
        -theta_e_dot (heading error rate)

        inputs:
        -delta (steering angle)

        published message:
        -steering_angle (radians)
        -speed (m/s)
        """

        self.get_logger().info("Here 1")
        # Update Car model LTV system --- A(Vx)
        sys = self.car_model.build_error_model(self.vx)

        # get updated gains
        K = self.update_gains()

        # Steering LQR
        self.get_logger().info("Here 2")
        steering_float_raw = -np.dot(K[0], self.state_est).flat[0]

        # Throttle gain scheduling
        tracking_error = self.state_measurement[0][0]
        self.inf_throttle = self.min_throttle - (self.min_throttle - self.max_throttle) / (1 - self.error_threshold)
        throttle_float_raw = ((self.min_throttle - self.max_throttle) / (1 - self.error_threshold)) * abs(tracking_error) + self.inf_throttle

        # Clamp control inputs
        # FIXME: need to convert to radians and m/s respectively 
        steering_float = self.clamp(steering_float_raw, self.max_right_steering, self.max_left_steering)
        throttle_float = self.clamp(throttle_float_raw, self.max_throttle, self.min_throttle)

        # Get Current Measurement
        self.y = self.car_model.calc_output(self.state_measurement)

        # Get optimal state estimates
        self.state_est, self.P = self.kalman_calc.lkf(sys, self.state_est, steering_float, self.y, self.P, self.Qo, self.Ro)
        self.get_logger().info("Here 3")

        # Publish values
        try:
            # publish drive control signal
            self.drive_cmd.header.stamp = self.current_time
            self.drive_cmd.header.frame_id = 'base_link'
            self.drive_cmd.drive.speed = speed
            self.drive_cmd.drive.steering_angle = delta
            self.drive_pub.publish(self.drive_cmd)

        except KeyboardInterrupt:
            self.drive_cmd.header.stamp = self.current_time
            self.drive_cmd.header.frame_id = 'base_link'
            self.drive_cmd.drive.speed = 0
            self.drive_cmd.drive.steering_angle = 0
            self.drive_pub.publish(self.drive_cmd)

        # Get new sensor measurements
        self.update_states()

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound is None:
            lower_bound = -upper_bound  # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c


def main(args=None):
    rclpy.init(args=args)
    lqg_publisher = LqgController()
    try:
        rclpy.spin(lqg_publisher)
        lqg_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        lqg_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        lqg_publisher.drive_cmd.header.stamp = lqg_publisher.current_time
        lqg_publisher.drive_cmd.header.frame_id = lqg_publisher.frame_id
        lqg_publisher.drive_cmd.drive.speed = 0.0
        lqg_publisher.drive_cmd.drive.steering_angle = 0.0
        lqg_publisher.drive_pub.publish(lqg_publisher.drive_cmd)
        time.sleep(1)
        lqg_publisher.destroy_node()
        rclpy.shutdown()
        lqg_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
