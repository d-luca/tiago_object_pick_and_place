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
from std_msgs.msg import Empty

target_red = 3
target_green = 2
target_blue = 1
phase_pick = 1

# links names
links_tag = {target_red: "tag_3",
             target_blue: "tag_1",
             target_green: "tag_2"}
# object link names in gazebo
links_gazebo = {target_red: "cube_link",
                target_blue: "Hexagon_link",
                target_green: "Triangle_link"}
# object model names in gazebo
models_gazebo = {target_red: "cube",
                 target_blue: "Hexagon",
                 target_green: "Triangle"}
pose_safe_move = Pose(Point(0.147546445981, 0.160143228025, 0.8192025537),
                      Quaternion(0.690661811861, 0.0232581242114, 0.722511873693, 0.0205405371673))
pose_safe_pick = Pose(Point(0.453058478764, -0.489125608369, 1.37155819656),
                      Quaternion(0.00990071387177, 0.338181114741, -0.0326650404228, 0.940461857085))
touch_links = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_grasping_frame"]


class PickServer(object):

    def __init__(self, name):
        # gazebo attach/detach servers
        self.gazebo_attach = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        # tf for frame transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        # moving parts
        self.client_torso = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client_gripper = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.robot = moveit_commander.RobotCommander()
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        # scene related
        self.scene = moveit_commander.PlanningSceneInterface()
        self.pub_start_spawn = rospy.Publisher("spawn_collision_objects", Tag_Spawn, queue_size=10)
        self.pub_stop_spawn = rospy.Publisher("stop_spawn", Empty, queue_size=10)
        rospy.sleep(0.1)
        # pick_place server
        self._as = actionlib.SimpleActionServer(name, Arm_MoveAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Pick server created")

    def add_z(self, pose, delta):
        new_pose = copy.deepcopy(pose)
        new_pose.position.z += delta
        return new_pose

    def lift_torso(self):
        self.client_torso.wait_for_server()
        self.feedback("[PcS] Moving torso up")
        waypoint = JointTrajectoryPoint(positions=[0.34], time_from_start=rospy.Duration(2))
        trajectory = JointTrajectory(joint_names=['torso_lift_joint'], points=[waypoint])
        self.client_torso.send_goal(FollowJointTrajectoryGoal(trajectory=trajectory))
        self.feedback("[PcS] Torso Goal sent")
        # Waits for the server to finish performing the action.
        wait = self.client_torso.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            self.abort("[PcS] Action server not available!")
            return
        return self.client_torso.get_result(), self.client_torso.get_state()

    def move_arm(self, pose):
        self.move_group_arm.set_pose_target(pose)
        plan = self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def safe_move(self):
        self.feedback("[PcS] Reaching the arm safe position to move around")
        self.move_arm(pose_safe_move)
        self.feedback("[PcS] Arm safe position to move around reached")
        rospy.sleep(0.5)

    def safe_pick(self):
        self.feedback("[PcS] Reaching the arm safe pick position")
        self.move_arm(pose_safe_pick)
        self.feedback("[PcS] Arm safe pick position reached")
        rospy.sleep(0.5)

    def move_arm_to_target(self, object_pose):
        orientate_eef = tran.quaternion_from_euler(0, np.pi / 2, 0, 'rxyz')
        object_orient = [object_pose.orientation.x, object_pose.orientation.y,
                         object_pose.orientation.z, object_pose.orientation.w]
        to_object = tran.quaternion_multiply(object_orient, orientate_eef)
        to_object = Quaternion(to_object[0], to_object[1], to_object[2], to_object[3])
        pose_goal = Pose(object_pose.position, to_object)
        self.feedback("[PcS] End effector pose to pick the object computed")
        self.move_arm(self.add_z(Pose(object_pose.position, to_object), 0.3))
        self.feedback("[PcS] End effector pose to pick the object reached")

    # z with respect to the frame of moveit "base_footprint"
    def linear_movement_z(self, delta):
        self.move_arm(self.add_z(self.move_group_arm.get_current_pose().pose, delta))

    def approach(self):
        self.feedback("[PcS] Approaching object")
        self.linear_movement_z(-0.07)
        self.feedback("[PcS] Object approached successfully")

    def depart(self):
        self.feedback("[PcS] Departing from object")
        self.linear_movement_z(0.07)
        self.feedback("[PcS] Successfully departed from object")

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
        self.feedback("[PcS] Gripper Goal sent!")
        # Waits for the server to finish performing the action.
        wait = self.client_gripper.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            self.abort("[PcS] Gripper server not available!")
            return
        # Result of executing the action
        return self.client_gripper.get_result(), self.client_gripper.get_state()

    def attach_gazebo(self, target_id):
        self.feedback("[PcS] Attaching object to Gazebo")
        req = AttachRequest()
        req.model_name_1 = "tiago"
        req.link_name_1 = "arm_7_link"
        req.model_name_2 = models_gazebo[target_id]
        req.link_name_2 = links_gazebo[target_id]
        self.gazebo_attach.wait_for_service()
        self.gazebo_attach.call(req)
        self.feedback("[PcS] Object attached to Gazebo")
        rospy.sleep(0.1)

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
        self.pub_start_spawn.publish(Tag_Spawn(target_id=target_id, phase=phase_pick))
        rospy.sleep(2)
        self.pub_stop_spawn.publish(Empty())

    def pick(self, target_id):
        result, status = self.lift_torso()
        if not status == actionlib.GoalStatus.SUCCEEDED:
            self.abort("[PcS] Error lifting the robot torso")
            return
        self.feedback("[PcS] Robot torso lifted correctly")

        # spawn object on peak position to avoid wrongly read frames from apriltag
        self.spawn_scene(target_id)
        self.safe_pick()
        # extract the taget object
        name = links_tag[target_id]
        dict_ret = self.scene.get_objects([name])
        collision_object = dict_ret[name]
        obj_pose = collision_object.primitive_poses[0]
        obj_shape = collision_object.primitives[0].dimensions
        obj_pose_stamped = PoseStamped(pose=copy.deepcopy(obj_pose))
        obj_pose_stamped.header.frame_id = "base_footprint"
        # pick object
        self.move_arm_to_target(obj_pose)
        self.open_gripper()
        self.approach()
        self.scene.remove_world_object(name)
        obj_pose_wrt_eef = self.transform_pose("arm_7_link", "base_footprint", obj_pose_stamped)
        self.attach_gazebo(target_id)
        self.close_gripper(0.07)
        self.depart()
        # re-add the object but with updated pose
        collision_object.primitive_poses[0] = self.transform_pose("base_footprint", "arm_7_link", obj_pose_wrt_eef).pose
        self.scene.add_object(collision_object)
        rospy.sleep(0.1)
        # attach the object to grasping frame with fingers that can touch it
        self.move_group_arm.attach_object(name, "gripper_grasping_frame", touch_links)
        # to safe position
        self.safe_pick()
        self.safe_move()
        # remove the scene but not the object picked
        # when the object is attached it is removed from known_objects and get_objects
        # and it's put in get_attached_objects
        # clear() would remove all the objects so also the picked one
        for obj in self.scene.get_known_object_names():
            self.scene.remove_world_object(obj)

    def execute_cb(self, goal):
        # helper variables
        self.pick(goal.target_id)
        rospy.sleep(0.1)
        self._as.set_succeeded(Arm_MoveResult(r="Succeeded"))


if __name__ == '__main__':
    rospy.init_node('pick_server')
    moveit_commander.roscpp_initialize(sys.argv)
    server = PickServer(rospy.get_name())
    rospy.spin()

