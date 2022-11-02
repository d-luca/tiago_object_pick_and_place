#! /usr/bin/env python
import numpy.random
import rospy
import random
import actionlib
from tiago_iaslab_simulation.srv import Objs, ObjsRequest, ObjsResponse
from pick_object.msg import MoveAction, MoveGoal, MoveResult, MoveFeedback
from pick_object.msg import Arm_MoveAction, Arm_MoveResult, Arm_MoveGoal, Arm_MoveFeedback

phase_pick = 1
phase_place = 2


class MainClient(object):

    def __init__(self):
        self.human_cl = rospy.ServiceProxy("human_objects_srv", Objs)
        self.move_cl = actionlib.SimpleActionClient('move_server', MoveAction)
        self.pick_cl = actionlib.SimpleActionClient('pick_server', Arm_MoveAction)
        self.place_cl = actionlib.SimpleActionClient('place_server', Arm_MoveAction)

    def get_target(self):
        rospy.loginfo("Waiting human service...")
        self.human_cl.wait_for_service()
        rospy.loginfo("Human service is online")
        request = ObjsRequest()
        request.ready = True
        request.all_objs = True
        response = self.human_cl.call(request)
        if response is not None:
            if len(response.ids) > 0:
                rospy.loginfo(str(response.ids)+" received from human node")
                return response.ids
        else:
            rospy.logwarn("Response from human node not received correctly")
        return

    def move_to_pick(self, target_id):
        self.move_cl.wait_for_server()
        self.move_cl.send_goal(MoveGoal(phase=phase_pick, target_id=target_id))
        wait = self.move_cl.wait_for_result()
        if not wait:
            rospy.logerr("Move server not available")
            return False
        result = self.move_cl.get_result()
        if result and self.move_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Motion to the pick position completed correctly")
        else:
            rospy.loginfo("Error in motion to the pick position")
            return False
        return True

    def move_to_place(self, target_id):
        self.move_cl.wait_for_server()
        self.move_cl.send_goal(MoveGoal(phase=phase_place, target_id=target_id))
        wait = self.move_cl.wait_for_result()
        if not wait:
            rospy.logerr("Move server not available")
            return False
        result = self.move_cl.get_result()
        if result and self.move_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Motion to the place position completed correctly")
        else:
            rospy.loginfo("Error in motion to the place position")
            return False
        return True

    def pick(self, target_id):
        self.pick_cl.wait_for_server()
        self.pick_cl.send_goal(Arm_MoveGoal(target_id=target_id))
        wait = self.pick_cl.wait_for_result()
        if not wait:
            rospy.logerr("Pick server not available")
            return False
        result = self.pick_cl.get_result()
        if result and self.pick_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Pick operations completed correctly")
        else:
            rospy.loginfo("Error in pick operations")
            return False
        return True

    def place(self, target_id):
        self.place_cl.wait_for_server()
        self.place_cl.send_goal(Arm_MoveGoal(target_id=target_id))
        wait = self.place_cl.wait_for_result()
        if not wait:
            rospy.logerr("Place server not available")
            return False
        result = self.place_cl.get_result()
        if result and self.place_cl.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Place operations completed correctly")
        else:
            rospy.loginfo("Error in place operations")
            return False
        return True


if __name__ == '__main__':
    try:
        rospy.init_node('main_client')
        main_cl = MainClient()

        target_id = main_cl.get_target()

        for i in range(len(target_id)):
            if target_id[i] is not None:
                if main_cl.move_to_pick(target_id[i]):
                    if main_cl.pick(target_id[i]):
                        if main_cl.move_to_place(target_id[i]):
                            if main_cl.place(target_id[i]):
                                pass

    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
