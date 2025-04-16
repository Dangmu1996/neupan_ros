import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from neupan import neupan

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
        self.scan_angle_range_para_ = self.get_parameter('scan_angle_range').value
        self.scan_downsample_ = self.get_parameter('scan_downsample').value
        self.scan_range_para_ = self.get_parameter('scan_range').value
        self.dune_checkpoint_ = self.get_parameter('dune_checkpoint').value
        self.refresh_initial_path_ = self.get_parameter('refresh_initial_path').value
        self.flip_angle_ = self.get_parameter('flip_angle').value

        pan = {'dune_checkpoint': self.dune_checkpoint_}
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file_, pan=pan
        )

        # data
        self.obstacle_points_ = None #(2,n) n number of points
        self.robot_state = None #(3,1)[x, y, theta]
        self.stop = False

        #publisher
        self.vel_pub_ = self.create_publisher(Twist, 'neupan_cmd_vel', 10)
        self.plan_pub_ = self.create_publisher(Path, 'neupan_plan', 10)
        self.ref_state_pub_ = self.create_publisher(Path,'neupan_ref_state', 10)

        # for visulaization
        self.point_markers_pub_dune_ = self.create_publisher(MarkerArray, 'dune_point_markers', 10)
        self.robot_marker_pub_ = self.create_publisher(Marker, 'robot_marker', 10)
        self.point_marker_pub__nrmp_ = self.create_publisher(MarkerArray, 'nrmp_point_markers')

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

    def scanCallback(self, msg):
        pass

    def pathCallback(self, msg):
        pass

    def goalCallback(self, msg):
        pass
    

def main(args=None):
    rclpy.init(args=args)
    nn = NeupanNode()

    rclpy.spin(nn)

    nn.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
