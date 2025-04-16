import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from neupan import neupan
from neupan.util import get_transform

import numpy as np
from math import sin, cos, atan2

class NeupanNode(Node):
    def __init__(self):
        super().__init__('neupan_node')

        # parameter
        self.declare_parameter('config_file','')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'laser_link')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_z', 1.0)
        self.declare_parameter('scan_angle_range',[-3.14, 3.14])
        self.declare_parameter('scan_downsample', 1)
        self.declare_parameter('scan_range', [0.0, 0.5])
        self.declare_parameter('dune_checkpoint','')
        self.declare_parameter('refresh_initial_path', False)
        self.declare_parameter('flip_angle', False)

        self.planner_config_file_ = self.get_parameter('config_file').value
        self.map_frame_ = self.get_parameter('map_frame').value
        self.base_frame_ = self.get_parameter('base_frame').value
        self.lidar_frame_ = self.get_parameter('lidar_frame').value
        self.marker_size_ = self.get_parameter('marker_size').value
        self.marker_z_ = self.get_parameter('marker_z').value
        self.scan_angle_range_= self.get_parameter('scan_angle_range').value
        self.scan_downsample_ = self.get_parameter('scan_downsample').value
        self.scan_range_ = self.get_parameter('scan_range').value
        self.dune_checkpoint_ = self.get_parameter('dune_checkpoint').value
        self.refresh_initial_path_ = self.get_parameter('refresh_initial_path').value
        self.flip_angle_ = self.get_parameter('flip_angle').value

        pan = {'dune_checkpoint': self.dune_checkpoint_}
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file_, pan=pan
        )

        # data
        self.obstacle_points_ = None #(2,n) n number of points
        self.robot_state_ = None #(3,1)[x, y, theta]
        self.stop_ = False

        #publisher
        self.vel_pub_ = self.create_publisher(Twist, 'neupan_cmd_vel', 10)
        self.plan_pub_ = self.create_publisher(Path, 'neupan_plan', 10)
        self.ref_state_pub_ = self.create_publisher(Path,'neupan_ref_state', 10)

        # for visulaization
        self.point_markers_pub_dune_ = self.create_publisher(MarkerArray, 'dune_point_markers', 10)
        self.robot_marker_pub_ = self.create_publisher(Marker, 'robot_marker', 10)
        self.point_marker_pub__nrmp_ = self.create_publisher(MarkerArray, 'nrmp_point_markers', 10)

        # tf listener
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # subscriber
        self.scan_sub_ = self.create_subscription(LaserScan,
                                                'scan',
                                                self.laserCallback,
                                                10)
        
        self.init_path_sub_ = self.create_subscription(Path,
                                                       'initial_path',
                                                       self.pathCallback,
                                                       10)
        
        self.neupan_goal_sub_ = self.create_subscription(PoseStamped,
                                                       'neupan_goal',
                                                       self.goalCallback,
                                                       10)
        


        self.timer_ = self.create_timer(0.02, self.run)
    
    def run(self):
        pass

    def laserCallback(self, msg):
        if self.robot_state_ is None:
            return None
        
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        points = []

        if self.flip_angle_:
            angles = np.flip(angles)
        
        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if(
                i % self.scan_downsample_ == 0
                and distance >= self.scan_range_[0]
                and distance <= self.scan_range_[1]
                and angle > self.scan_angle_range_[0]
                and angle < self.scan_angle_range_[1]
            ):
                point = np.array([[distance * cos(angle)],[distance * sin(angle)]])
                points.append(point)
        
        if len(points) == 0:
            self.obstacle_points_ = None
            self.get_logger().info("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer_.lookup_transform(self.map_frame_, self.lidar_frame_, now)
            t = trans.transform.translation
            
            x, y = t.x, t.y

            r = trans.transform.rotation
            rotation = [r.x, r.y, r.z, r.w]

            yaw = self.quat_to_yaw_list(rotation)
            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            return self.obstacle_points
            
        except TransformException as ex:
            self.get_logger().warn(f"Transform failed: {ex}")
            return

    def pathCallback(self, msg):
        pass

    def goalCallback(self, msg):
        pass
    
    def quat_to_yaw_list(self, quater):
        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

def main(args=None):
    rclpy.init(args=args)
    nn = NeupanNode()

    rclpy.spin(nn)

    nn.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
