import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import time
import math
import os
import numpy as np

NODE_NAME = 'pid_node'
ERROR_TOPIC_NAME = '/error'
ACTUATOR_TOPIC_NAME = '/drive'

PATH_TOPIC_NAME = '/global_trajectory'
ODOM_TOPIC_NAME = '/ego_racecar/odom'

class PidController(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.QUEUE_SIZE = 10

        self.path_thread = MutuallyExclusiveCallbackGroup()
        self.odom_thread = MutuallyExclusiveCallbackGroup()

        # Actuator control
        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()

        # Get Odometry measurements
        self.odom_subscriber = self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odom_measurement, 10, callback_group=self.odom_thread)
        self.odom_subscriber

        # # Error subscriber
        # self.error_subscriber = self.create_subscription(Float32MultiArray, ERROR_TOPIC_NAME, self.error_measurement, self.QUEUE_SIZE)
        # self.error_subscriber

        # Get Reference Trajectory
        self.path_subscriber = self.create_subscription(Path, PATH_TOPIC_NAME, self.set_path, QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, depth=50), callback_group=self.path_thread)
        self.path_subscriber

        # setting up message structure for vesc-ackermann msg
        self.current_time = self.get_clock().now().to_msg()
        self.frame_id = 'base_link'

        # Default actuator values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_steering', 1),
                ('Ki_steering', 0),
                ('Kd_steering', 0),
                ('integral_max', 0),
                ('heading_upper_error_threshold', 0.15),
                ('heading_lower_error_threshold', 0.15),
                ('long_upper_error_threshold', 0.15),
                ('long_lower_error_threshold', 0.15),
                ('zero_speed', 0.0),
                ('max_speed', 5),
                ('min_speed', 0.1),
                ('max_right_steering', 0.4),
                ('max_left_steering', -0.4)
            ])
        self.Kp = self.get_parameter('Kp_steering').value
        self.Ki = self.get_parameter('Ki_steering').value
        self.Kd = self.get_parameter('Kd_steering').value
        self.integral_max = self.get_parameter('integral_max').value 
        self.heading_upper_error_threshold = self.get_parameter('heading_upper_error_threshold').value # between [0,1]
        self.heading_lower_error_threshold = self.get_parameter('heading_lower_error_threshold').value # between [0,1]
        self.long_upper_error_threshold = self.get_parameter('long_upper_error_threshold').value # between [0,1]
        self.long_lower_error_threshold = self.get_parameter('long_lower_error_threshold').value # between [0,1]
        self.zero_speed=self.get_parameter('zero_speed').value  # should be around 0
        self.max_speed=self.get_parameter('max_speed').value  # between [0,5] m/s
        self.min_speed=self.get_parameter('min_speed').value  # between [0,5] m/s 
        self.max_right_steering=self.get_parameter('max_right_steering').value  # negative(max_left) 
        self.max_left_steering=self.get_parameter('max_left_steering').value  # between abs([0,0.436332]) radians (0-25degrees)

        # initializing PID control
        self.x = 0
        self.y = 0
        self.x_buffer = 0
        self.y_buffer = 0

        # self.x_path = []
        # self.y_path = []
        self.x_path = np.array([1.0, 3.0, 5.0])
        self.y_path = np.array([1.0, 3.0, 5.0])
        self.line_error_threshold = 0.001

        self.e_y_buffer = 0
        self.e_x_buffer = 0
        self.e_cg_buffer = 0
        self.e_theta_buffer = 0
        self.e_y = 0
        self.e_y_1 = 0
        self.e_x = 0
        self.e_cg = 0
        self.e_theta = 0

        self.proportional_error = 0 # proportional error term for steering
        self.derivative_error = 0 # derivative error term for steering
        self.integral_error = 0 # integral error term for steering
        
        self.get_logger().info(
            f'\n Kp_steering: {self.Kp}'
            f'\n Ki_steering: {self.Ki}'
            f'\n Kd_steering: {self.Kd}'
            f'\n heading_upper_error_threshold: {self.heading_upper_error_threshold}'
            f'\n heading_lower_error_threshold: {self.heading_lower_error_threshold}'
            f'\n long_upper_error_threshold: {self.long_upper_error_threshold}'
            f'\n long_lower_error_threshold: {self.long_lower_error_threshold}'
            f'\n zero_speed: {self.zero_speed}'
            f'\n max_speed: {self.max_speed}'
            f'\n min_speed: {self.min_speed}'
            f'\n max_right_steering: {self.max_right_steering}'
            f'\n max_left_steering: {self.max_left_steering}'
        )
        # Call controller
        self.Ts = 0.01  # contoller sample time
        self.create_timer(self.Ts, self.controller)

        self.clock = time.perf_counter()
        self.sub_working = False

    def odom_measurement(self, odom_data):
        # TODO: what is frequency of data coming in?

        # car position
        self.x_buffer = odom_data.pose.pose.position.x
        self.y_buffer = odom_data.pose.pose.position.y

        # car orientation
        # FIXME: confirm coordinate axes
        # quaternion = (odom_data.orientation.x, odom_data.orientation.y, odom_data.orientation.z, odom_data.orientation.w)
        # euler = euler_from_quaternion(quaternion)
        # self.yaw_buffer = euler[2]

        # # car velocity
        # self.vx_buffer = odom_data.twist.twist.linear.x
        # self.vy_buffer = odom_data.twist.twist.linear.y

    # def error_measurement(self, error_data):
    #     error_data_check = np.array([error_data.data[0], error_data.data[1], error_data.data[2]])
    #     if not (np.isnan(error_data_check).any()):
    #         self.e_y_buffer = error_data.data[0]
    #         self.e_x_buffer = error_data.data[1]
    #         self.e_theta_buffer = error_data.data[2]

    def set_path(self, path_data):
        # TODO: Currently not working with Lidar Nav

        # path orientation 
        # FIXME: confirm coordinate axes
        # quaternion = (path_data.poses[0].pose.orientation.x, path_data.poses[0].pose.orientation.y,
        #               path_data.poses[0].pose.orientation.z, path_data.poses[0].pose.orientation.w)
        # euler = euler_from_quaternion(quaternion)
        # self.yaw_path = euler[2]

        # path coordinates (GLOBAL)
        self.x_path = np.array([pose.pose.position.x for pose in path_data.poses])
        self.y_path = np.array([pose.pose.position.y for pose in path_data.poses])
        self.get_logger().info(f"first val PATH (x): ({self.x_path[-1]})")

    def get_cross_track_error(self):
        # self.get_logger().info("Updating CROSS-TRACK-ERROR")
        
        # find 2 closest points in path with car
        error_x = self.x_path - self.x
        error_y = self.y_path - self.y
        error_mag = np.power(np.power(error_x,2) + np.power(error_y, 2), 0.5)
        error_mag1, error_mag2 = np.partition(error_mag, 1)[0:2]
        self.get_logger().info(f"{len(self.x_path)}")
        error_mag1_index = np.argwhere(error_mag == error_mag1)[0][0]
        error_mag2_index = np.argwhere(error_mag == error_mag2)[0][0]
        Px1 = self.x_path[error_mag1_index]
        Px2 = self.x_path[error_mag2_index]
        Py1 = self.y_path[error_mag1_index]
        Py2 = self.y_path[error_mag2_index]

        self.get_logger().info(f"Car pos: {self.x},{self.y}")
        self.get_logger().info(f"Path data: {self.x_path[:5]},{self.y_path[:5]}")
        self.get_logger().info(f"Err Mag index: {error_mag1_index},{error_mag2_index}")
        self.get_logger().info(f"Point 1: {Px1},{Py1} \nPoint 2: {Px2},{Py2}")
        
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

        return ecg_x, ecg_y, e_cg, theta_path

    def get_latest_measurements(self):

        # car coordinates
        self.x = self.x_buffer
        self.y = self.y_buffer

        ecg_x, ecg_y, e_cg, theta_path = self.get_cross_track_error()

        self.e_y_buffer = ecg_y
        self.e_x_buffer = ecg_x
        self.e_cg_buffer = e_cg
        self.e_theta_buffer = theta_path

        self.e_y = self.e_y_buffer
        self.e_x = self.e_x_buffer
        self.e_cg = self.e_cg_buffer
        self.e_theta = self.e_theta_buffer
        self.current_time = self.get_clock().now().to_msg()

    def controller(self):
        
        # Wait for subscriber to start up
        if len(self.x_path) < 4:
            self.get_logger().info(f'\n Waiting for path subscriber to start')
        else:
            if not self.sub_working:
                res = time.perf_counter()
                self.get_logger().info(f'\n'
                                f'\n time to start sub:{res - self.clock}')
                self.sub_working = True

            # Get latest measurement
            self.get_latest_measurements()

            # Steering PID terms
            self.proportional_error = self.Kp * self.e_cg
            self.derivative_error = self.Kd * (self.e_cg - self.e_y_1) / self.Ts
            self.integral_error += self.Ki * self.e_cg * self.Ts
            self.integral_error = self.clamp(self.integral_error, self.integral_max)
            delta_raw = self.proportional_error# + self.derivative_error + self.integral_error

            # Throttle gain scheduling (function of error)
            self.inf_throttle = self.min_speed - ((self.min_speed - self.max_speed) / (self.heading_upper_error_threshold - self.heading_lower_error_threshold)) * self.heading_upper_error_threshold
            speed_raw = ((self.min_speed - self.max_speed) / (self.heading_upper_error_threshold - self.heading_lower_error_threshold)) * abs(self.e_theta) + self.inf_throttle

            # clamp values
            delta = self.clamp(delta_raw, self.max_right_steering, self.max_left_steering)
            speed = self.clamp(speed_raw, self.max_speed, self.min_speed)
            
            self.get_logger().info(f'\n'
                                f'\n x:{self.x}'
                                f'\n y:{self.y}'
                                f'\n ex:{self.e_x}'
                                f'\n ey:{self.e_y}'
                                f'\n ecg:{self.e_cg}'
                                f'\n e_theta:{self.e_theta}'
                                f'\n delta:{delta_raw}'
                                f'\n speed:{speed_raw}'
                                f'\n clamped delta:{delta}'
                                f'\n clamped speed:{speed}'
                                )
            self.e_y_1 = self.e_cg

            # Publish values to VESC

            if self.e_cg < self.long_upper_error_threshold:
                # Publish values
                try:
                    # publish drive control signal
                    self.drive_cmd.header.stamp = self.current_time
                    self.drive_cmd.header.frame_id = self.frame_id
                    self.drive_cmd.drive.speed = speed
                    # self.drive_cmd.drive.steering_angle = -delta
                    self.drive_cmd.drive.steering_angle = delta
                    self.drive_pub.publish(self.drive_cmd)

                except KeyboardInterrupt:
                    self.drive_cmd.header.stamp = self.current_time
                    self.drive_cmd.header.frame_id = self.frame_id
                    self.drive_cmd.drive.speed = 0
                    self.drive_cmd.drive.steering_angle = 0
                    self.drive_pub.publish(self.drive_cmd)
            else:
                # publish drive control signal
                self.drive_cmd.header.stamp = self.current_time
                self.drive_cmd.header.frame_id = self.frame_id
                self.drive_cmd.drive.speed = self.zero_speed
                self.drive_cmd.drive.steering_angle = 0.0
                self.drive_pub.publish(self.drive_cmd)

    def clamp(self, value, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if value < lower_bound:
            value_c = lower_bound
        elif value > upper_bound:
            value_c = upper_bound
        else:
            value_c = value
        return value_c 


def main(args=None):
    rclpy.init(args=args)
    pid_publisher = PidController()
    try:
        rclpy.spin(pid_publisher)
        pid_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        pid_publisher.drive_cmd.header.stamp = pid_publisher.current_time
        pid_publisher.drive_cmd.header.frame_id = pid_publisher.frame_id
        pid_publisher.drive_cmd.drive.speed = 0.0
        pid_publisher.drive_cmd.drive.steering_angle = 0.0
        pid_publisher.drive_pub.publish(pid_publisher.drive_cmd)
        time.sleep(1)
        pid_publisher.destroy_node()
        rclpy.shutdown()
        pid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
