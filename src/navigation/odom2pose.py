#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu, MagneticField

from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Odom2pose:

    def __init__(self):
        # Creates a node with name 'odom2pose' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('odom2pose', anonymous=True)

        # Publisher which will publish to the topic '/odom_pose'.
        self.pose_odom_publisher = rospy.Publisher('/pose_odom',
                                              PoseStamped, queue_size=10)

        # Publisher which will publish to the topic '/gyro_pose'.
        self.pose_gyro_publisher = rospy.Publisher('/pose_gyro',
                                              PoseStamped, queue_size=10)

        # Publisher which will publish to the topic '/odom_pose'.
        self.pose_mag_publisher = rospy.Publisher('/pose_mag',
                                              PoseStamped, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = rospy.Subscriber('/sensor_state',
                                                 SensorState, self.update_odom)

        # A subscriber to the topic '/imu'. self.update_pose is called
        # when a message of type Imu is received.
        self.gyro_subscriber = rospy.Subscriber('/imu',
                                                Imu, self.update_gyro)

        # A subscriber to the topic '/magnetic_field'. self.update_pose is called
        # when a message of type Imu is received.
        self.mag_subscriber = rospy.Subscriber('/magnetic_field',
                                                MagneticField, self.update_mag)

        self.pose_odom = np.array([0.0,0.0,0.0]) # [x, y, theta]
        self.pose_gyro = np.array([0.0,0.0,0.0]) # [x, y, theta]
        self.pose_mag = np.array([0.0,0.0,0.0]) # [x, y, theta]
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0

        self.encoder_resolution = 4096
        self.wheel_radius = 0.033
        self.wheel_separation = 0.160
        self.delta_t = 0.01
        self.mag_offset = 0.0 # To do
        self.flag_first_data = True

        # If we press control + C, the node will stop.
        rospy.spin()

    def update_odom(self, data):
        """Callback function which is called when a new message of type SensorState is
        received by the subscriber."""
        if self.flag_first_data:
            self.prev_left_encoder = data.left_encoder
            self.prev_right_encoder = data.right_encoder
            self.flag_first_data = False
            return

        # To do
        # Compute pose self.pose_odom = np.array([x, y, theta])
        # from data.left_encoder, self.prev_left_encoder, data.right_encoder and self.prev_right_encoder
        # using self.encoder_resolution, self.wheel_radius and self.wheel_separation

        vit_d = ((data.right_encoder - self.prev_right_encoder) * 2 * np.pi) / (self.encoder_resolution * self.delta_t)
        vit_g = ((data.left_encoder - self.prev_left_encoder) * 2 * np.pi) / (self.encoder_resolution * self.delta_t)
        v = self.wheel_radius * (vit_g + vit_d) / 2
        w = self.wheel_radius * (vit_g - vit_d) / (self.wheel_separation)
        self.pose_odom[0] += self.delta_t * v * np.cos(self.pose_odom[2])
        self.pose_odom[1] += self.delta_t * v * np.sin(self.pose_odom[2])
        self.pose_odom[2] += self.delta_t * w


        self.prev_right_encoder = data.right_encoder
        self.prev_left_encoder = data.left_encoder

        # update position gyro self.pose_gyro[0] and self.pose_gyro[1]
        # using orientation self.pose_gyro[2] and ds from odometry

        self.pose_gyro[0] += self.delta_t * v * np.cos(self.pose_gyro[2])
        self.pose_gyro[1] += self.delta_t * v * np.sin(self.pose_gyro[2]) 

        # update position mag self.pose_mag[0] and self.pose_mag[1]
        # using orientation self.pose_mag[2] and ds from odometry

        self.pose_mag[0] += self.delta_t * v * np.cos(self.pose_mag[2])
        self.pose_mag[1] += self.delta_t * v * np.sin(self.pose_mag[2]) 

        # self.pose_odom publication
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = self.pose_odom[0]
        poseStamped.pose.position.y = self.pose_odom[1]
        [poseStamped.pose.orientation.x,
        poseStamped.pose.orientation.y,
        poseStamped.pose.orientation.z,
        poseStamped.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, self.pose_odom[2])
        poseStamped.header = data.header
        poseStamped.header.frame_id = 'odom'
        self.pose_odom_publisher.publish(poseStamped)

    def update_gyro(self, data):
        # To do
        # Compute self.pose_gyro[2] = theta
        # from angular velocity data.angular_velocity.z
        # using self.delta_t

        self.pose_gyro[2] += data.angular_velocity.z * self.delta_t

        # self.pose_gyro publication
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = self.pose_gyro[0]
        poseStamped.pose.position.y = self.pose_gyro[1]
        [poseStamped.pose.orientation.x,
        poseStamped.pose.orientation.y,
        poseStamped.pose.orientation.z,
        poseStamped.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, self.pose_gyro[2])
        poseStamped.header = data.header
        poseStamped.header.frame_id = 'odom'
        self.pose_gyro_publisher.publish(poseStamped)

    def update_mag(self, data):
        # To do
        # Compute self.pose_mag[2] = theta
        # from magnetic field data.magnetic_field.x, data.magnetic_field.y and data.magnetic_field.z
        # using self.mag_offset (To correct manually)

        self.pose_mag[2] = np.arctan2(data.magnetic_field.y, data.magnetic_field.x)

        # self.pose_mag publication
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = self.pose_mag[0]
        poseStamped.pose.position.y = self.pose_mag[1]
        [poseStamped.pose.orientation.x,
        poseStamped.pose.orientation.y,
        poseStamped.pose.orientation.z,
        poseStamped.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, self.pose_mag[2])
        poseStamped.header = data.header
        poseStamped.header.frame_id = 'odom'
        self.pose_mag_publisher.publish(poseStamped)

if __name__ == '__main__':
    try:
        Odom2pose()
    except rospy.ROSInterruptException:
        pass
