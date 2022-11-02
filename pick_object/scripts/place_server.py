#! /usr/bin/env python

import sys
import copy
import numpy as np
import tf.transformations as tran
import tf2_ros
import rospy
import actionlib
import moveit_commander
from tf2_geometry_msgs import do_transform_pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from pick_object.msg import Arm_MoveAction, Arm_MoveResult, Arm_MoveFeedback, Tag_Spawn
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

target_red = 3
target_green = 2
target_blue = 1
phase_place = 2

# links names
links_tag = {target_blue: "tag_1", target_green: "tag_2", target_red: "tag_3"}

# object link names in gazebo
links_gazebo = {target_blue: "Hexagon_link",
                target_green: "Triangle_link",
                target_red: "cube_link"}

models_gazebo = {target_blue: "Hexagon",
                 target_green: "Triangle",
                 target_red: "cube"}

collision_cylinder = {target_blue: "blue_cylinder",
                      target_green: "green_cylinder",
                      target_red: "red_cylinder"}

pose_safe_move = Pose(Point(0.147546445981, 0.160143228025, 0.8192025537),
                      Quaternion(0.690661811861, 0.0232581242114, 0.722511873693, 0.0205405371673))
pose_safe_pick = Pose(Point(0.453058478764, -0.489125608369, 1.37155819656),
                      Quaternion(0.00990071387177, 0.338181114741, -0.0326650404228, 0.940461857085))


class PlaceServer(object):

    def __init__(self, name):
        # gazebo attach/detach servers
        self.gazebo_detach = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.gazebo_detach.wait_for_service()
        # tf for frame transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        # moving parts
        self.client_gripper = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client_gripper.wait_for_server()
        self.client_torso = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.robot = moveit_commander.RobotCommander()
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        # scene related
        self.scene = moveit_commander.PlanningSceneInterface()
        self.pub_start_spawn = rospy.Publisher("spawn_collision_objects", Tag_Spawn, queue_size=10)

        self._as = actionlib.SimpleActionServer(name, Arm_MoveAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Place server created")

    def add_z(self, pose, delta):
        new_pose = copy.deepcopy(pose)
        new_pose.position.z += delta
        return new_pose

    def abort(self, message):
        rospy.logerr(message)
        self._as.set_aborted(Arm_MoveResult(r=message))

    def succeed(self, message):
        rospy.loginfo(message)
        self._as.set_succeeded(Arm_MoveResult(r=message))

    def feedback(self, message):
        rospy.loginfo(message)
        self._as.publish_feedback(Arm_MoveFeedback(f=message))

    def spawn_scene(self, target_id):
        self.pub_start_spawn.publish(Tag_Spawn(target_id=target_id, phase=phase_place))
        rospy.sleep(0.1)

    def move_arm(self, pose):
        self.move_group_arm.set_pose_target(pose)
        plan = self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def safe_move(self):
        self.feedback("[PlS] Reaching the arm safe position to move around")
        self.move_arm(pose_safe_move)
        self.feedback("[PlS] Arm safe position to move around reached")
        rospy.sleep(0.5)

    def safe_place(self):
        self.feedback("[PlS] Reaching the arm safe position for placing the object")
        self.move_arm(pose_safe_pick)
        self.feedback("[PlS] Arm safe position for placing the object reached")
        rospy.sleep(0.5)

    def approach(self, dz):
        waypoints = []
        scale = 1
        wpose = self.move_group_arm.get_current_pose().pose
        wpose.position.z -= scale * dz  # Move down along z
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group_arm.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        self.move_group_arm.execute(plan, wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()
        # return plan, fraction

    def depart(self, dz):
        waypoints = []
        scale = 1

        wpose = self.move_group_arm.get_current_pose().pose
        wpose.position.z += scale * dz # Move down along z
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group_arm.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        self.move_group_arm.execute(plan, wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()
        # return plan, fraction

    def close_gripper(self, distance):
        return self.move_gripper(distance/2, distance/2)

    def open_gripper(self):
        return self.move_gripper(0.25, 0.25)

    def move_gripper(self, left, right):
        self.client_gripper.wait_for_server()
        waypoint = JointTrajectoryPoint(positions=[left, right], time_from_start=rospy.Duration(2))
        trajectory = JointTrajectory(joint_names=['gripper_left_finger_joint', 'gripper_right_finger_joint'],
                                     points=[waypoint])
        self.client_gripper.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
        self.feedback("[PlS] Gripper Goal sent!")
        # Waits for the server to finish performing the action.
        wait = self.client_gripper.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            self.abort("[PlS] Gripper server not available!")
            return
        # Result of executing the action
        return self.client_gripper.get_result(), self.client_gripper.get_state()

    def transform_pose(self, target_frame, start_frame, pose):
        transform_ok = False
        ret_pose = None
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfBuffer.lookup_transform(target_frame, start_frame, rospy.Time(0))
                ret_pose = do_transform_pose(pose, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.01)
        return ret_pose

    def detach(self, target_id):
        self.feedback("[PlS] Detaching object")
        req = AttachRequest()
        req.model_name_1 = "tiago"
        req.link_name_1 = "arm_7_link"
        req.model_name_2 = models_gazebo[target_id]
        req.link_name_2 = links_gazebo[target_id]
        self.gazebo_detach.wait_for_service()
        self.gazebo_detach.call(req)
        rospy.sleep(0.1)
        self.scene.remove_attached_object(name=links_tag[target_id], link="gripper_grasping_frame")
        self.feedback("[PlS] Object detached")

    def place(self, target_id):
        self.spawn_scene(target_id)
        self.lift_torso()
        dz = 0.15
        obj_ret = self.scene.get_attached_objects([links_tag[target_id]])
        obj_obj = obj_ret[links_tag[target_id]]
        obj_pose = obj_obj.object.primitive_poses[0]
        obj_shape = obj_obj.object.primitives[0].dimensions

        dict_ret_2 = self.scene.get_objects([collision_cylinder[target_id]])
        cylinder = dict_ret_2[collision_cylinder[target_id]]
        cylinder_pose = cylinder.primitive_poses[0]
        cylinder_shape = cylinder.primitives[0].dimensions

        cylinder_pose.position.z += cylinder_shape[0]/2 #because the cylinder link is in its center
        cylinder_pose.position.z += obj_shape[0] #because about 1/2 of the object will be external to the arm

        self.safe_place()
        self.move_arm_to_target(cylinder_pose)
        rospy.sleep(1)#only for debug or if there could be swinging
        self.approach(dz)
        rospy.sleep(1)#only for debug or if there could be swinging
        self.open_gripper()
        self.detach(target_id)
        rospy.sleep(0.1)
        self.depart(dz)
        self.safe_move()#don't know if we should also go to the safe_place before
        self.scene.clear()

    def lift_torso(self):
        self.client_torso.wait_for_server()
        rospy.loginfo("[PlS] Moving torso up")
        trajectory = JointTrajectory()
        trajectory.joint_names = ['torso_lift_joint']
        waypoint = JointTrajectoryPoint()
        waypoint.positions = [0.34]
        waypoint.time_from_start = rospy.Duration(2)
        trajectory.points.append(waypoint)

        # Creates a new goal with the MoveBaseGoal constructor
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory

        # Sends the goal to the action server.
        self.client_torso.send_goal(goal)

        rospy.loginfo("[PlS] torso Goal sent!")
        # Waits for the server to finish performing the action.
        wait = self.client_torso.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("[PlS] Action server not available!")
            rospy.signal_shutdown("[PlS] Action server not available!")
            return
        # Result of executing the action
        result = self.client_torso.get_result()
        if result and self.client_torso.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[PlS] torso moved correctly!")
        else:
            rospy.loginfo("[PlS] error in torso movement!")

    def move_arm_to_target(self, object_pose):
        orientate_eef = tran.quaternion_from_euler(0, np.pi / 2, 0, 'rxyz')
        object_orient = [object_pose.orientation.x, object_pose.orientation.y,
                         object_pose.orientation.z, object_pose.orientation.w]
        to_object = tran.quaternion_multiply(object_orient, orientate_eef)
        to_object = Quaternion(to_object[0], to_object[1], to_object[2], to_object[3])
        pose_goal = Pose(object_pose.position, to_object)
        self.feedback("[PlS] End effector pose to pick the object computed")
        self.move_arm(self.add_z(Pose(object_pose.position, to_object), 0.3))
        self.feedback("[PlS] End effector pose to pick the object reached")

    def execute_cb(self, goal):
        # helper variables
        self.place(goal.target_id)
        rospy.sleep(0.1)
        self._as.set_succeeded(Arm_MoveResult(r="Succeeded"))


if __name__ == '__main__':
    rospy.init_node('place_server')
    moveit_commander.roscpp_initialize(sys.argv)
    server = PlaceServer(rospy.get_name())
    rospy.spin()

