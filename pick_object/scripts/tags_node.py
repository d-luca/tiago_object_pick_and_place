#!/usr/bin/env python
import copy
import numpy as np
import sys
import rospy
import tf2_ros
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tran
from std_msgs.msg import Empty
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPlanningSceneRequest, GetPlanningScene
from tf2_geometry_msgs import do_transform_pose
from pick_object.msg import Tag_Spawn


# 110%
slack = 1.1
# 0.05 cube
square_edge = 0.05
# x4 cubes
big_exahedron_height = 4*square_edge
# 1 cube
big_exahedron_radius = square_edge
# x2 cubes
small_exahedron_height = 2*square_edge
# half cube
small_exahedron_radius = 0.03
# 1 cube
pyramid_edge = square_edge
table_edge = 1
table_height = 0.785
table_x = 7.765
table_y = -2.97

red_cylinder_x = 4.007396 + 6.504857 #gazebo pose + difference betw gazebo and rviz reference frames
green_cylinder_x = 5.007404 + 6.504857 #gazebo pose + difference betw gazebo and rviz reference frames
blue_cylinder_x = 6.007146 + 6.504857 #gazebo pose + difference betw gazebo and rviz reference frames
cylinder_y = 1.015966 - 1.32815
cylinder_z = 0.344999

big_cylinder_height = 0.69+0.02
big_cylinder_radius = 0.21+0.04

target_red = 3
target_green = 2
target_blue = 1
phase_pick = 1
phase_place = 2
prism_list = [target_blue, 4, 5, 6, 7]


class Tags(object):

    def __init__(self):
        # parameters for the adding process
        self.adding_is_on = False
        self.added_list = list()
        # getting access to the scene
        self.scene = moveit_commander.PlanningSceneInterface()
        self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        rospy.loginfo("[TN] Waiting for scene service")
        self.scene_srv.wait_for_service()
        rospy.loginfo("[TN] Scene service connected")
        # tf for frame conversion
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        # initialize the scene
        self.initialize()
        # subscribe to the topics
        self.sub_april = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.april_callback)
        self.sub_spawn_on = rospy.Subscriber("spawn_collision_objects", Tag_Spawn, self.spaw_collision_objects_callback)
        self.sub_spawn_off = rospy.Subscriber("stop_spawn", Empty, self.stop_spaw_callback)

    def wait_for_planning_scene_object(self, object_name):
        rospy.loginfo("[TN] Waiting for object '" + object_name + "' to appear in planning scene...")
        gps_req = GetPlanningSceneRequest()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
        part_in_scene = False
        while not rospy.is_shutdown() and not part_in_scene:
            # This call takes a while when rgbd sensor is set
            gps_resp = self.scene_srv.call(gps_req)
            # check if 'part' is in the answer
            for collision_obj in gps_resp.scene.world.collision_objects:
                if collision_obj.id == object_name:
                    part_in_scene = True
                    break
            else:
                rospy.sleep(1.0)
        rospy.loginfo("[TN] '" + object_name + "' is in scene!")

    def transform_pose(self, pose, target_frame, start_frame):
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfBuffer.lookup_transform(target_frame, start_frame, rospy.Time(0))
                map_pose = do_transform_pose(pose, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.01)
        return map_pose

    def add_z(self, pose, delta_z, subtraction_frame):
        # in map frame that has z pointing up
        out_pose = self.transform_pose(pose, subtraction_frame, pose.header.frame_id)
        # subtract z
        out_pose.pose.position.z += delta_z
        # back to the original frame
        out_pose = self.transform_pose(out_pose, pose.header.frame_id, subtraction_frame)
        out_pose.header.stamp = self.tfBuffer.get_latest_common_time(pose.header.frame_id, subtraction_frame)
        return out_pose

    def add_table(self):
        # if already present updates the box
        name = "table"
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.pose.position.x = table_x
        object_pose.pose.position.y = table_y
        object_pose.pose.position.z = table_height/2
        object_pose.pose.orientation.w = 1.0
        object_pose.header.frame_id = "map"
        object_pose.header.stamp = rospy.Time.now()
        self.scene.add_box(name, object_pose, size=[table_edge, table_edge, table_height])
        self.wait_for_planning_scene_object(name)

    def add_triangle(self, tag):
        # if already present updates the box
        name = "tag_"+str(tag.id[0])
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.pose = copy.deepcopy(tag.pose.pose.pose)
        object_pose.header.frame_id = tag.pose.header.frame_id
        # ------#### position
        # center of the base
        object_pose = self.add_z(object_pose, -square_edge/2, "tag_2")
        # center of the prism
        object_pose = self.add_z(object_pose, np.sqrt(2)/2*square_edge/2, "base_footprint")
        # ------#### orientation
        # rotate pi/4 around x to have z pointing up
        orientation = tag.pose.pose.pose.orientation
        quat = np.asarray([orientation.x, orientation.y, orientation.z, orientation.w])
        # put the oriention parallel to the table
        rotation = tran.quaternion_from_euler(np.pi/4, 0, 0, "rxyz")
        quat = tran.quaternion_multiply(quat, rotation)
        object_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        # ------#### add to the scene
        square_size = [square_edge*slack*slack, np.sqrt(2)*square_edge*slack, np.sqrt(2)/2*square_edge*slack]
        self.scene.add_box(name, object_pose, size=square_size)
        self.wait_for_planning_scene_object(name)

    def add_box(self, tag):
        # if already present updates the box
        name = "tag_"+str(tag.id[0])
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.pose.position = tag.pose.pose.pose.position
        object_pose.pose.orientation = tag.pose.pose.pose.orientation
        object_pose.header.frame_id = tag.pose.header.frame_id
        object_pose = self.add_z(object_pose, -square_edge/2, name)
        square_size = [square_edge*slack, square_edge*slack, square_edge*slack]
        self.scene.add_box(name, object_pose, size=square_size)
        self.wait_for_planning_scene_object(name)

    def add_cylinder(self, tag):
        identifier = tag.id[0]
        name = "tag_"+str(identifier)
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.pose = tag.pose.pose.pose
        object_pose.header.frame_id = tag.pose.header.frame_id
        if identifier == 1:
            object_pose = self.add_z(object_pose, -small_exahedron_height/2, name)
            self.scene.add_cylinder(name, object_pose, height=small_exahedron_height*slack,
                               radius=small_exahedron_radius*slack)
        else:
            object_pose = self.add_z(object_pose, -big_exahedron_height/2, name)
            self.scene.add_cylinder(name, object_pose, height=big_exahedron_height*slack, radius=big_exahedron_radius*slack)
        self.wait_for_planning_scene_object(name)

    def add_cylinder_table(self, target_id):
        if target_id == target_blue:
            self.add_general_cylinder("blue_cylinder", "place_table_b", blue_cylinder_x, cylinder_y,
                                      cylinder_z, big_cylinder_height, big_cylinder_radius)
        elif target_id == target_green:
            self.add_general_cylinder("green_cylinder", "place_table_g", green_cylinder_x, cylinder_y,
                                      cylinder_z, big_cylinder_height, big_cylinder_radius)
        elif target_id == target_red:
            self.add_general_cylinder("red_cylinder", "place_table_r", red_cylinder_x, cylinder_y,
                                      cylinder_z, big_cylinder_height, big_cylinder_radius)

    def add_general_cylinder(self, name, identifier, x, y, z, height, radius):
        pose = PoseStamped()
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        self.scene.add_cylinder(name, pose, height=height, radius=radius)
        # check if the box is known but not attached
        self.wait_for_planning_scene_object(name)

    def initialize(self):
        self.added_list = list()
        rospy.sleep(0.1)

    def spaw_collision_objects_callback(self, message):
        if message.target_id not in [target_blue, target_green, target_red]:
            rospy.logerr("[TN] Received target {"+str(message.target_id)+"} is not implemented")
            return
        if message.phase == phase_pick:
            self.initialize()
            self.adding_is_on = True
            self.add_table()
            rospy.loginfo("[TN] Spawn from Apriltag started")
        elif message.phase == phase_place:
            self.add_cylinder_table(message.target_id)
        else:
            rospy.logerr("[TN] Received phase {"+str(message.phase)+"} is not implemented")

    def stop_spaw_callback(self, empty):
        self.adding_is_on = False
        rospy.logwarn("[TN] Spawn from Apriltag stopped")

    def april_callback(self, tag_array):
        if self.adding_is_on:
            for tag in tag_array.detections:
                if tag.id not in self.added_list:
                    self.added_list.append(tag.id)
                    if tag.id[0] in prism_list:
                        self.add_cylinder(tag)
                    elif tag.id[0] == target_red:
                        self.add_box(tag)
                    elif tag.id[0] == target_green:
                        self.add_triangle(tag)


if __name__ == "__main__":
    rospy.init_node("tags")
    moveit_commander.roscpp_initialize(sys.argv)
    node = Tags()
    # spinning the callbacks
    rospy.spin()
