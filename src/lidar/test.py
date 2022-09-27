import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu, MagneticField

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
            PointField('c', 12, PointField.INT16, 1)]

class Test:

    def __init__(self):
        # Creates a node with name 'odom2pose' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('test', anonymous=True)

        # Publisher which will publish to the topic '/odom_pose'.
        self.pose_odom_publisher = rospy.Publisher('/pose_odom',
                                              PoseStamped, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = rospy.Subscriber('/sensor_state',
                                                 SensorState, self.update_odom)

        pub_pc2 = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, update_carto)

        print("coucou")

        #Constantes
        self.encoder_resolution = 4096
        self.wheel_radius = 0.033
        self.wheel_separation = 0.160
        self.delta_t = 0.01
        self.mag_offset = np.pi/2.0-0.07
        self.flag_first_data = True

        self.largeurCarto = 400 #cm
        self.largeurBloc = 10 #cm


        #LIDAR
        self.mouvPossible = False
        self.cartographie = np.full((self.largeurCarto/self.largeurBloc, self.largeurCarto/self.largeurBloc), -1) #-1 for unknow, 0 for free, 1 for obstacle

        #ODOM
        self.pose_odom = np.array([4.0,4.0,0.0]) # [x, y, theta]
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0


        # If we press control + C, the node will stop
        rospy.spin()

    def update_odom(self, data) :
        ds = 0.0

        left_delta_angle = ((data.left_encoder-self.prev_left_encoder)/float(self.encoder_resolution))*2*np.pi
        right_delta_angle = ((data.right_encoder-self.prev_right_encoder)/float(self.encoder_resolution))*2*np.pi

        left_ds = left_delta_angle*self.wheel_radius
        right_ds = right_delta_angle*self.wheel_radius

        ds = (right_ds+left_ds)/2.0
        da = (right_ds-left_ds)/float(self.wheel_separation)

        self.pose_odom[2] += da
        self.pose_odom[0] += ds*np.cos(self.pose_odom[2])
        self.pose_odom[1] += ds*np.sin(self.pose_odom[2])

        self.prev_right_encoder = data.right_encoder
        self.prev_left_encoder = data.left_encoder

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

    def update_carto(self, data) :
        coords = []
        objectTooClose = False

        for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        if msg.ranges[i] < 0.1:
            objectTooClose = True
            continue

        # ToDo: Polar to Cartesian transformation
        coords.append([self.pose[0]+msg.ranges[i]*np.cos(theta+self.pose[2]), self.pose[1]+msg.ranges[i]*np.sin(theta+self.pose[2])])

        self.mouvPossible = objectTooClose

        # Create a PointCloud2 message from coordinates
        pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])
        pub_pc2.publish(pc2)


if __name__ == '__main__':
    try:
        Test()
    except rospy.ROSInterruptException:
        pass