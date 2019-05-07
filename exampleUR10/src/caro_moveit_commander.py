#!/usr/bin/env python

import copy
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
import time

#=============================          CLASS          =================================

class MoveCommanderUr10 (object): 
    """ Move commander created by CLP with the template of MoveGroupPythonInterfaceTutorial"""

    def __init__(self): 

        super(MoveCommanderUr10, self).__init__() # Is this necessary?

        #SETUP
        moveit_commander.roscpp_initialize (sys.argv) # Initialize the moveit commander
        
        self.robot_comm = moveit_commander.RobotCommander() #Initiate a RobotCommander object
        self.scene_interface = moveit_commander.PlanningSceneInterface() #Initiate a Planning scene Interface object

        group_name = "right_arm_and_manipulator" #Initiate a Move Group Commander object 
        self.group = moveit_commander.MoveGroupCommander(group_name)

        #Publisher to publish trajectories for RViz to visualize: 
        self.display_trajectory_publisher = rospy.Publisher("/move_group", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link= self.group.get_end_effector_link()
        self.group_names = self.robot_comm.get_group_names()
        self.box_name= ""
        self.box_pose = geometry_msgs.msg.PoseStamped()    

#===========================      PLAN       =========================================

    def plan_to_pose_target (self, position_x, position_y, position_z, orient_x, orient_y, orient_z, orient_w):

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


    def execute_plan (self, plan):
        
        self.group.execute(plan, wait=True)


    # def go_to_pregrasp_position (self, object_pose, pregrasp_distance=0.3):
    #     """ It should be already a box added to the scene"""
            
    #     if type (object_pose) is not geometry_msgs.msg.PoseStamped(): 
    #         rospy.logerr ( "The object pose should be a geometry_msgs.msg.PoseStamped()")
    #         return 

    #     else:  
    #         wpose = object_pose
    #         wpose.position_z += 0.3 #Go to 30 cm above the box 
    #         plan_to_pose_target (self,  object_pose.position.x, object_pose.position.y, object_pose.position.z, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)    


#============================       BOX     =============================================

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
        
        self.scene_interface.add_box(self.box_name, self.box_pose, size= (0.1, 0.1, 0.1)) 

    def attach_box(self, timeout=4):
        touch_links = ["hand_base_attach"]
        self.scene_interface.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

    def detach_box(self, timeout= 4):
        
        self.scene_interface.remove_attached_object(self.eef_link, name=self.box_name)
    
    def remove_box(self, timeout=4):
        
        self.scene_interface.remove_world_object(self.box_name)
#===============================================================================================================
#                                              Main   
#=============================================================================================================
        
if __name__ == '__main__':
    
    rospy.init_node("move_commander_ur10", anonymous= True) # Define the node

    ur10_commander= MoveCommanderUr10()    
#     #Random pose
    
   
         
    # Pose to vertical axis (i)
    

     # Pose to horizontal axis (1)
   

    # # Pose to horizontal axis (1)
    # plan_horiz1= ur10_commander.plan_to_pose_target(1.2768, 0.36727, 0.26319,-0.0092237,0.707,0.7071 ,0.0084781)
     
    # if len(plan_horiz1.joint_trajectory.points)>0: 
    #     print "Plan to -1 "
    #     raw_input("if Plan is ok. press enter to execute")
  
    # else: 
    #     rospy.logerr ( "Fail to make a plan")

    # ur10_commander.execute_plan(plan_horiz1)
    

    #Go to a defined joint state

    plan_with_joints = ur10_commander.go_to_joint_target(-0.03883398363815905, -1.5276687999544085, 1.5388371215293597, 3.1372948604019957, 0.039511801410542426, 3.134863975410523)
    
    if len(plan_with_joints.joint_trajectory.points)>0: 
        print "Plan to joint state "
        raw_input("if Plan is ok. press enter to execute")
  
    else: 
        rospy.logerr ( "Fail to make a plan")

    ur10_commander.execute_plan(plan_with_joints)

    #Add a box
      

   
    
    
