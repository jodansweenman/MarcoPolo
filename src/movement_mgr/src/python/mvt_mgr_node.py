#!/usr/bin/env python

import rospy
#from movement_mgr.msg import ExploreTaskAction, ExploreTaskGoal
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header, String
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
import tf2_ros
import actionlib
import numpy as np

class MovementMgr():
    def __init__(self):
        rospy.loginfo('Initializing Marco Movement Manager')

        # Initialize tf listener for later lookup during pursuit
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # Subscribe to message which will indicate ready to pursue
        rospy.loginfo('Subscribing to /marco/pursue_cube')
        rospy.Subscriber("/marco/pursue_cube", String, self.change_to_pursuit)

        # Startup movement clients and wait for servers to come online
        self.fe_client = actionlib.SimpleActionClient('explore_server', ExploreTaskAction)
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #TODO: re-enable waiting for the server
        rospy.loginfo("waiting for explore_server and move_base servers")
        #self.fe_client.wait_for_server()
        #self.mb_client.wait_for_server()

        # Create initial goal handle which will be sent to frontier exploration
        self.init_gh = self._create_initial_frontier_goal()

        # Send Exploration goal
        rospy.loginfo("Sending initial frontier goal from movement_manager")
        self.fe_client.send_goal(self.init_gh)
        return

    def change_to_pursuit(self, data):
        rospy.loginfo("Entered callback to change to pursuit")
        frame_name = data.data
        rospy.loginfo("Cancelling Frontier exploration goals")
        self.fe_client.cancel_all_goals()
        #manual_gh = self._create_naive_pursuit_goal()
        manual_gh = self._create_pursuit_goal()
        self.mb_client.send_goal(manual_gh)
        return
    
    def _create_pursuit_goal(self):
        
        # Initialize goal which will go to move_base for positioning near ball
        mb_goal = MoveBaseGoal()
        target_pose = PoseStamped()

        # Note that pose will be provided in the map frame
        target_pose.header.frame_id = "map"
        curr_stamp = rospy.Time.now()
        target_pose.header.stamp = curr_stamp

        # Get Transformations between robot base_footprint and map, and cube and map
        try:
            # get latest tf for cube
            cube_tf = self.tfBuffer.lookup_transform("map",
                                                     "ball_position",
                                                     rospy.Time(0))

            # get current tf for bot
            base_fp_tf = self.tfBuffer.lookup_transform("map",
                                                        "base_footprint",
                                                        curr_stamp,
                                                        timeout=rospy.Duration(secs=1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Creation of pursuit goal failed on TF lookup")
            rospy.logerr(e)

        base_fp_vec = np.array([base_fp_tf.transform.translation.x,
                                base_fp_tf.transform.translation.y]).reshape(-1, 1)
        
        cube_vec = np.array([cube_tf.transform.translation.x,
                             cube_tf.transform.translation.y]).reshape(-1, 1)


        goal_pt, yaw = self._get_pt_theta(base_fp_vec, cube_vec)
        target_pose.pose.position = Point(x=goal_pt[0, 0], y=goal_pt[1, 0], z=0.0)
        target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw))
        rospy.loginfo(target_pose)

        mb_goal.target_pose = target_pose
        return mb_goal

    def _get_pt_theta(self, R, C, base_rad_m=0.2):
        """
        Creates a pt = [x, y]' vector and yaw scalar
        from two input vectors representing points
        in the map frame.
        Param:
        - R = [x_bot, y_bot]' (m)
        - C = [x_cube, y_cube]' (m)
        Return:
        - pt = [x_target, y_target]' for bot in world frame (m)
        - theta = yaw rotation in world frame (rad)
        """
        #rospy.loginfo("Received R: %s, C: %s" % (R, C))
        G = C - R
        #rospy.loginfo("Calculated G = %s" % G)
        G_mag = np.linalg.norm(G)
        #rospy.loginfo("Calculated G_mag = %s" % G_mag)

        # magnitude of distance for goal is magnitude of distance
        # between points - radius of base
        G_p_mag = G_mag - base_rad_m 
        #rospy.loginfo("Then G_p_mag = %s" % G_p_mag)
        gx, gy = G[0,0], G[1, 0]
        #rospy.loginfo("gx is %s, gy is %s" % (gx, gy))
        theta = np.arctan(gy/gx)
        # Handle cases where tangent wraps around
        if gx < 0.0:
            theta += np.pi
        #rospy.loginfo("Then theta is %s radians (%s degrees)" % (theta, np.rad2deg(theta)))
        G_p = G_p_mag * (np.array([np.cos(theta), np.sin(theta)]).reshape(-1, 1))
        #rospy.loginfo("G_p is %s" % G_p)
        pt = R + G_p
        #rospy.loginfo("Finally, pt is %s" % pt)
        #rospy.loginfo("Determined pt = %s and theta = %s" % (pt, theta))

        return pt, theta

    def _create_naive_pursuit_goal(self):
        mb_goal = MoveBaseGoal()
        curr_stamp = rospy.Time.now()
        target_pose = PoseStamped()
        target_pose.header.stamp = curr_stamp
        target_pose.header.frame_id = "map"
        target_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
        
        mb_goal.target_pose = target_pose

        rospy.loginfo(mb_goal)
        return mb_goal

    def _create_initial_frontier_goal(self):
        frontier_gh = ExploreTaskGoal()
        curr_ts = rospy.Time.now()

        curr_header = Header()
        curr_header.frame_id = "map"
        curr_header.seq = 1
        curr_header.stamp = curr_ts
        
        # Point Stamped
        pt_s = PointStamped()
        pt_s.header = curr_header
        pt_s.point.x = 1.0
        # Polygon Stamped
        # Note that polygon is not defined so it's an unbounded exploration
        pg_s = PolygonStamped()
        pg_s.header = curr_header
        vertices = [(5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0), (5.0, -5.0)]
        for vertex in vertices:
            pg_s.polygon.points.append(Point(x=vertex[0], y=vertex[1]))

        #frontier_gh.header = curr_header
        #frontier_gh.goal_id.stamp = curr_ts
        #frontier_gh.goal_id.id = 'initial_frontier_marco'
        frontier_gh.explore_boundary = pg_s
        frontier_gh.explore_center = pt_s
        
        rospy.loginfo(frontier_gh)  
        return frontier_gh  

def main():

    rospy.init_node(name='movement_mgr')
    MovementMgr()
    rospy.loginfo("Created and now spinning")
    rospy.spin()

    return

if __name__ == "__main__":
    main()
