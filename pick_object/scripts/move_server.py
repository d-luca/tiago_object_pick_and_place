#! /usr/bin/env python

import rospy
import actionlib
import pick_object
import intro_navigation
from intro_navigation.msg import NavigationAction, NavigationGoal
from pick_object.msg import MoveAction, MoveResult, MoveFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

target_red = 3
target_green = 2
target_blue = 1
phase_pick = 1
phase_place = 2

head_pos_pick = {target_red: [0.4, -0.7],
                 target_blue: [0.27, -0.7],
                 target_green: [-0.2, -0.7]}
head_pose_place = [0, -0.3]
inter_pick_pose = Pose(Point(8.319, 0.072, 0.0), Quaternion(0.0, 0.0, -0.763, 0.646))
inter_place_pose = Pose(Point(8.700, -0.143, 0.0), Quaternion(0.0, 0.0, 0.589, 0.808))
poses_pick = {target_blue: Pose(Point(8.6, -2.6, 0.0), Quaternion(0.0, 0.0, 1.0, 0.018)),
              target_green: Pose(Point(7.7, -3.9, 0.0), Quaternion(0.0, 0.0, 0.703, 0.711)),
              target_red: Pose(Point(7.4, -2, 0.0), Quaternion(0.0, 0.0, -0.673, 0.739))}
poses_place = {target_blue: Pose(Point(12.527, 0.329, 0.0), Quaternion(0.0, 0.0, -0.760, 0.646)),
               target_green: Pose(Point(11.514, 0.292, 0.0), Quaternion(0.0, 0.0, -0.760, 0.646)),
               target_red: Pose(Point(10.557, 0.386, 0.0), Quaternion(0.0, 0.0, -0.763, 0.646))}


class MoveServer(object):

    def __init__(self, name):
        self.client_move = actionlib.SimpleActionClient('our_server', NavigationAction)
        self.client_head = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        self._as = actionlib.SimpleActionServer(name, MoveAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        print("Move action server created")

    def reach_position(self, pose):
        # Waits until the action server has started up and started listening for goals.
        self.client_move.wait_for_server()
        # Creates a new goal with the MoveBaseGoal constructor
        goal = NavigationGoal()
        # position
        goal.xp = pose.position.x
        goal.yp = pose.position.y
        goal.wz = pose.orientation.z
        goal.ww = pose.orientation.w

        # Sends the goal to the action server.
        self.client_move.send_goal(goal)
        rospy.loginfo("[MS] Goal sent!")
        # Waits for the server to finish performing the action.
        wait = self.client_move.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            self.abort("[MS] Move_base server doesn't reply")
        else:
            # Result of executing the action
            return self.client_move.get_result(), self.client_move.get_state()

    def move_head(self, joint1, joint2):
        # Waits until the action server has started up and started listening for goals.
        self.client_head.wait_for_server()
        rospy.loginfo("[MS] Moving head")

        waypoint = JointTrajectoryPoint(positions=[joint1, joint2], time_from_start=rospy.Duration(2))
        trajectory = JointTrajectory(joint_names=['head_1_joint', 'head_2_joint'], points=[waypoint])
        goal = FollowJointTrajectoryGoal(trajectory=trajectory)

        # Sends the goal to the action server.
        self.client_head.send_goal(goal)

        rospy.loginfo("[MS] Head Goal sent!")
        # Waits for the server to finish performing the action.
        wait = self.client_head.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            self.abort("[MS] Error server is not available")
            return
        # Result of executing the action
        return self.client_head.get_result(), self.client_head.get_state()

    def reset_head(self):
        return self.move_head(0, -0.3)

    def abort(self, message):
        rospy.logerr(message)
        self._as.set_aborted(MoveResult(r=message))

    def succeed(self, message):
        rospy.loginfo(message)
        self._as.set_succeeded(MoveResult(r=message))

    def feedback(self, message):
        rospy.loginfo(message)
        self._as.publish_feedback(MoveFeedback(f=message))

    def move_to_pick(self, target_id):
        # head standard position
        result, status = self.reset_head()
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching standard head pose on pick phase")
            return
        self.feedback("[MS] Standard head pose on pick phase reached")
        # inter pos
        result, status = self.reach_position(inter_pick_pose)
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching intermediate position on pick phase")
            return
        self.feedback("[MS] Intermediate position on pick phase reached")
        # final pos
        result, status = self.reach_position(poses_pick[target_id])
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching the final position on pick phase")
            return
        self.feedback("[MS] Final position on pick phase reached")
        # head final pos
        result, status = self.move_head(head_pos_pick[target_id][0], head_pos_pick[target_id][1])
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching the final head pose on pick phase")
            return
        self.feedback("[MS] Final head pose on pick phase reached")
        self.succeed("[MS] Motion to target completed")

    def move_to_place(self, target_id):
        # head standard position
        result, status = self.reset_head()
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching standard head pose on place phase")
            return
        self.feedback("[MS] Standard head pose on place phase reached")
        # inter pos
        result, status = self.reach_position(inter_place_pose)
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching intermediate position on place phase")
            return
        self.feedback("[MS] Intermediate position on place phase reached")
        # final pos
        result, status = self.reach_position(poses_place[target_id])
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching the final position on place phase")
            return
        self.feedback("[MS] Final position on place phase reached")
        # head final pos
        result, status = self.move_head(head_pose_place[0], head_pose_place[1])
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[MS] Error reaching the final head pose on place phase")
            return
        self.feedback("[MS] Final head pose on place phase reached")
        self.succeed("[MS] Motion to target completed")

    def execute_cb(self, goal):
        goal_id = goal.target_id
        goal_phase = goal.phase
        if goal_id not in [target_blue, target_green, target_red]:
            self.abort("Target {"+str(goal_id)+"} not implemented")
            return

        if goal_phase == phase_pick:
            self.move_to_pick(goal_id)
        elif goal_phase == phase_place:
            self.move_to_place(goal_id)
        else:
            self.abort("Phase {"+str(goal_phase)+"} not implemented")


if __name__ == '__main__':
    rospy.init_node('move_server')
    server = MoveServer(rospy.get_name())
    rospy.spin()
