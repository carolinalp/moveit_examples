#!/usr/bin/env python
import sys
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy


class MoveCommanderUr10 (object): 
    """ Move commander created by CLP with the template of MoveGroupPythonInterfaceTutorial"""

    def __init__(self): 

        # SETUP
        # Initialize the moveit commander
        moveit_commander.roscpp_initialize(sys.argv) 
        # Initiate a RobotCommander object
        self.robot_comm = moveit_commander.RobotCommander()
        # Initiate a Planning scene Interface object
        self.scene_interface = moveit_commander.PlanningSceneInterface()  
        # Initiate a Move Group Commander object 
        group_name = "right_arm_and_manipulator"  
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # Publisher to publish trajectories for RViz to visualize: 
        self.display_trajectory_publisher = rospy.Publisher("/move_group", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot_comm.get_group_names()
        self.box_name = ""
        self.box_pose = geometry_msgs.msg.PoseStamped()    

    def plan_to_pose_target(self, position_x, position_y, position_z, orient_x, orient_y, orient_z, orient_w):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = position_x
        pose_goal.position.y = position_y
        pose_goal.position.z = position_z
        pose_goal.orientation.x = orient_x
        pose_goal.orientation.y = orient_y
        pose_goal.orientation.z = orient_z
        pose_goal.orientation.w = orient_w
        
        self.group.set_pose_target(pose_goal)

        plan = self.group.plan()

        return plan

    def go_to_joint_target (self,shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3):
        """shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 are the position
        values of these joints"""

        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = shoulder_pan
        joint_goal[1] = shoulder_lift
        joint_goal[2] = elbow
        joint_goal[3] = wrist_1
        joint_goal[4] = wrist_2
        joint_goal[5] = wrist_3

        plan = self.group.plan(joint_goal)
        return plan

    def execute_plan(self, plan):
        
        self.group.execute(plan, wait=True)

    def add_box(self, position_x, position_y, position_z, timeout=4):
        """Adding Objects to the Planning Scene
        position_x, position_y, position_z are floats. The position in the
        planning scene where you want the box"""
        
        self.box_name = "box"

        self.box_pose.header.frame_id = "world"
        self.box_pose.pose.position.x = position_x
        self.box_pose.pose.position.y = position_y
        self.box_pose.pose.position.z = position_z
        self.box_pose.pose.orientation.w = 1.0
        
        self.scene_interface.add_box(self.box_name, self.box_pose, size =(0.1, 0.1, 0.1))

    def attach_box(self, timeout=4):
        touch_links = ["hand_base_attach"]
        self.scene_interface.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    def detach_box(self, timeout=4): 
        self.scene_interface.remove_attached_object(self.eef_link, name=self.box_name)

    def remove_box(self, timeout=4):
        self.scene_interface.remove_world_object(self.box_name)
        
