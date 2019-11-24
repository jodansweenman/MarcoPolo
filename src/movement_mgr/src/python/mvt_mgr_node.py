#!/usr/bin/env python

import rospy
from movement_mgr.msg import ExploreTaskAction, ExploreTaskActionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
import actionlib

class MovementMgr():
    def __init__(self):
        rospy.loginfo('Initializing Marco Movement Manager')
        rospy.Subscriber("/marco/pursue_cube", Bool, self.change_to_pursuit)
        self.fe_client = actionlib.SimpleActionClient('marco_rq_exploration', ExploreTaskAction)
        self.mb_client = actionlib.SimpleActionClient('marco_rq_pursuit', MoveBaseAction)
        #TODO: re-enable waiting for the server
        #self.fe_client.wait_for_server()
        #self.mb_clien.wait_for_server()
        self.manual_gh = None
        self.init_gh = self._create_initial_frontier_goal()
        self.send_fe_goal(self.init_gh)

    def send_fe_goal(self, goal):
        rospy.loginfo("Sending initial frontier goal from movement_manager")
        self.fe_client.send_goal(goal)
        return
    
    def change_to_pursuit(self, data):
        rospy.loginfo("Entered callback to change to pursuit")
        if data.data == True:
            self.fe_client.cancel_all_goals()
            self.manual_gh = self._create_pursuit_goal('ball_position')
            self.mb_client.send_goal(self.manual_gh)
        return

    # TODO: Make this actually create the correct pose
    def _create_pursuit_goal(self, cube_tf_framename):
        mb_goal = MoveBaseActionGoal()
        curr_stamp = rospy.Time.now()
        target_pose = PoseStamped()
        # TODO: replace "odom" with cube_tf_framename
        target_pose.header.stamp = curr_stamp
        target_pose.header.frame_id = "odom"
        target_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
        

        mb_goal.header = target_pose.header
        mb_goal.goal.target_pose = target_pose
        mb_goal.goal_id.stamp = curr_stamp
        mb_goal.goal_id.id = "manual pursuit"
        rospy.loginfo(mb_goal)
        return mb_goal

    def _create_initial_frontier_goal(self):
        frontier_gh = ExploreTaskActionGoal()
        curr_ts = rospy.Time.now()

        curr_header = Header()
        curr_header.frame_id = "odom"
        curr_header.seq = 1
        curr_header.stamp = curr_ts
        
        # Point Stamped
        pt_s = PointStamped()
        pt_s.header = curr_header
        # Polygon Stamped
        # Note that polygon is not defined so it's an unbounded exploration
        pg_s = PolygonStamped()
        pg_s.header = curr_header

        frontier_gh.header = curr_header
        frontier_gh.goal_id.stamp = curr_ts
        frontier_gh.goal_id.id = 'initial_frontier_marco'
        frontier_gh.goal.explore_boundary = pg_s
        frontier_gh.goal.explore_center = pt_s
        
        rospy.loginfo(frontier_gh)  
        return frontier_gh  

def main():

    rospy.init_node(name='movement_mgr')
    mvmt = MovementMgr()

    rospy.spin()

    return

if __name__ == "__main__":
    main()
