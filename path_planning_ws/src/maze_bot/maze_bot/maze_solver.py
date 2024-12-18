import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_pathplanning import bot_pathplanner
import numpy as np

from nav_msgs.msg import Odometry
from .bot_motionplanning import bot_motionplanner

class maze_solver(Node):
    def __init__(self):
        super().__init__("maze_solving_node")
        self.velocity_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)

        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()
        self.vel_msg=Twist()
        self.bot_localizer = bot_localizer()  
        self.bot_mapper = bot_mapper()  
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner() 

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.bot_motionplanner.get_pose,10)
        self.sat_view = np.zeros((100,100))

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view = frame
        cv2.imshow("sat_view", self.sat_view)
        cv2.waitKey(1)

    def maze_solving(self):
        frame_disp = self.sat_view.copy()
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)
        self.bot_mapper.graphify(self.bot_localizer.maze_og)
        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze, method="dijisktra")
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze, method="a_star")
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze, method="DFS")
        # self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze, method="DFS_Shortest")
        print("\nNodes visited [Dijisktra V A-Star*] = [{} V {}]".format(self.bot_pathplanner.dijisktra.dijiktra_nodes_visited, self.bot_pathplanner.astar.astar_nodes_visited))
        cv2.waitKey(0)    

        # bot_loc = self.bot_localizer.loc_car
        # path = self.bot_pathplanner.path_to_goal
        # self.bot_motionplanner.nav_path(bot_loc, path, self.vel_msg, self.velocity_publisher)

        # img_shortest_path = self.bot_pathplanner.img_shortest_path
        # self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp)
        # cv2.imshow("Maze (Live)", frame_disp)
        # cv2.waitKey(1)

def main(args =None):
    rclpy.init()
    node_obj =maze_solver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 docker.io/library/getting-started1 bash
# docker exec -it r2_pathplanning_container bash
# ros2 launch maze_bot maze_1_robot_camera.launch.py
# ros2 run maze_bot maze_solver
# 
# # docker build -t getting-started .