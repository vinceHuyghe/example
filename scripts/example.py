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
from std_msgs.msg import Header
from ur_msgs.srv import SetIO
from yaml import safe_load
import pyassimp
from shape_msgs.msg import Mesh, MeshTriangle

import os

absolute_path = os.path.dirname(__file__)
relative_path = "ur_ee.stl"
path = os.path.join(absolute_path, relative_path)

PLANNING_GROUP = "manipulator"

class TrajectoryHandler:

    def __init__(self, sim: bool = True) -> None:

        moveit_commander.roscpp_initialize(argv)
        rospy.init_node('tarjectory_handler', anonymous=True)
        rospy.loginfo(f'{rospy.get_name()} started')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.
            DisplayTrajectory, queue_size=20)
        self.move_group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
        
        
        # Setup planning scene
        # self.move_group.set_workspace((-1.5 , -1.5 , -0.2, 2, 1.5, 1.5))
        self.ee_name = 'end_effector'
        self.attach_endeffector(self.ee_name)
        rospy.sleep(0.2)
        
        rospy.loginfo('waiting for get_planning_scene service')
        rospy.wait_for_service('/get_planning_scene', 30)
        self.get_planning_scene_service = rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)
        
        planning_scene_req = moveit_msgs.msg.PlanningSceneComponents()
        planning_scene_req.components = 0
        resp = self.get_planning_scene_service(planning_scene_req)       
        self.scene.apply_planning_scene(resp)
        rospy.sleep(0.2)
        self.move_group.set_end_effector_link(f'{self.ee_name}/tcp')
        rospy.loginfo(
            f'end effector link set to {self.move_group.get_end_effector_link()}')
        
        
    def attach_endeffector(self,
                        name: str,
                        filename = path 
                        ) -> bool:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.move_group.get_end_effector_link()
        pose.pose = Pose(position=Point(0, 0, 0),
                        orientation=Quaternion(0, 0, 1, 0))

        ee = moveit_msgs.msg.AttachedCollisionObject()
        ee.object = self.make_mesh(name, pose, filename=filename)
        ee.object.subframe_names = ['tcp']
        ee.object.subframe_poses = [Pose(
            position=Point(0.184935, 0, 0.06),
            orientation=Quaternion(0, 0, 1, 0))]
        ee.link_name = self.move_group.get_end_effector_link()

        self.scene.attach_object(ee)

        return self.wait_for_state_update(name, object_is_attached=True)
    
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
    
    @staticmethod
    def make_mesh(name: str, pose: PoseStamped, filename: str, scale=(1, 1, 1)):

        co = moveit_msgs.msg.CollisionObject()
        if pyassimp is False:
            rospy.logerr(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt"
            )
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            rospy.logerr("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            rospy.logerr("There are no faces in the mesh")
        co.operation = moveit_msgs.msg.CollisionObject.ADD
        co.id = name
        co.header = pose.header
        co.pose = pose.pose

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, "__len__"):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, "indices"):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [
                        face.indices[0],
                        face.indices[1],
                        face.indices[2],
                    ]
                    mesh.triangles.append(triangle)
        else:
            rospy.logerr(
                "Unable to build triangles from mesh due to mesh object structure"
            )
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        co.meshes = [mesh]
        pyassimp.release(scene)
        return co
    
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

    pose = Pose(
        position=Point(0.6, 0.5, 0.4),
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
   
