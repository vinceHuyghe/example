#!/usr/bin/env python3

import random
from math import pi
from sys import argv
from typing import List

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPlanningScene
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


PLANNING_GROUP = "manipulator"


class TrajectoryHandler:

    def __init__(self) -> None:

        moveit_commander.roscpp_initialize(argv)
        rospy.init_node('tarjectory_handler', anonymous=True)
        rospy.loginfo(f'{rospy.get_name()} started')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
        rospy.sleep(0.2)
        self.ee_name = 'end_effector'
        self.attach_endeffector(self.ee_name)
        

        rospy.loginfo('waiting for get_planning_scene service')
        rospy.wait_for_service('/get_planning_scene', 30)
        self.get_planning_scene_service = rospy.ServiceProxy(
            '/get_planning_scene', GetPlanningScene)

        planning_scene_req = moveit_msgs.msg.PlanningSceneComponents()
        planning_scene_req.components = 0
        resp = self.get_planning_scene_service(planning_scene_req)
        self.scene.apply_planning_scene(resp)
        rospy.sleep(0.2)
        self.move_group.set_end_effector_link(f'{self.ee_name}/tcp')
        rospy.loginfo(
            f'end effector link set to {self.move_group.get_end_effector_link()}')

    def attach_endeffector(self, name: str) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'link_6'
        pose.pose = Pose(position=Point(0.1, 0, 0),
                         orientation=Quaternion(0, 0, 1, 0))

        ee = moveit_msgs.msg.AttachedCollisionObject()
        ee.link_name = 'link_6'
        size = (0.2, 0.2, 0.2)
        ee.object = self.make_box(name, pose, size)
        ee.object.subframe_names = ['tcp']
        ee.object.subframe_poses = [Pose(
            position=Point(0.184935, 0, 0.06),
            orientation=Quaternion(0, 0, 1, 0))]
        ee.touch_links = ['link_6']
        

        self.scene.attach_object(ee)

        return self.wait_for_state_update(name, object_is_attached=True)

    @staticmethod
    def make_box(name, pose, size):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        return co

    def wait_for_state_update(self,
                              object_name: str,
                              object_is_attached: bool = False,
                              object_is_known: bool = False,
                              timeout: int = 4) -> bool:

        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = object_name in self.scene.get_known_object_names()

            if (object_is_attached == is_attached) and (
                    object_is_known == is_known):
                rospy.loginfo(f'{object_name} added to scene')
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def create_goal(self, joint_values: List[float]) -> JointState:

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = 'base_link'
        joint_state.name = self.move_group.get_active_joints()
        joint_state.position = joint_values

        return joint_state


def planner_example():

    th = TrajectoryHandler()

    th.move_group.set_planning_pipeline_id('ompl')

    th.move_group.set_joint_value_target(
        th.create_goal((0.0, 0.0, 00, 0.0, 0.0, 0.0)))
    plan = th.move_group.plan()
    th.move_group.go(wait=True)

    pose = Pose(
        position=Point(0.6, -0.5, 0.4),
        orientation=Quaternion(0.0, 1.0, 0.0, 0.0))
    th.move_group.set_pose_target(pose)
    plan = th.move_group.plan()

    th.move_group.go(wait=True)


def main():
    try:
        planner_example()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
