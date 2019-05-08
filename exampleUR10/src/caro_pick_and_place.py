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

from caro_moveit_commander import MoveCommanderUr10

class PickAndPlace (object):
    
    """ This class moves the robot to initial positions, add a box in position1
    grasp the box and move it to the position2, then deatach the box and remove 
    it. Assumes that position1 and position2 are elements of the type 
    geometry_msgs.msg.PoseStamped """
    

    def __init__(self, position1, position2):
        
        self.ur10_commander= MoveCommanderUr10()
        self.position1= position1
        self.position2 = position2
        self.go_to_home_position()           
        self.ur10_commander.add_box(self.position1.pose.position.x, self.position1.pose.position.y, self.position1.pose.position.z )
        time.sleep (2)         
        self.go_to_pregrasp_position(self.position1)
        time.sleep (2)
        self.go_to_grasp_position()
        time.sleep (5) 
        self.ur10_commander.remove_box()
        

    def go_to_home_position(self): 

        plan_with_joints = self.ur10_commander.go_to_joint_target(-0.03883398363815905, -1.5276687999544085, 1.5388371215293597, 3.1372948604019957, 0.039511801410542426, 3.134863975410523)
    
        if len(plan_with_joints.joint_trajectory.points)>0: 
            print "Plan to joint state "
            raw_input("if Plan is ok. press enter to execute")

        else: 
            rospy.logerr ( "Fail to make a plan")

        self.ur10_commander.execute_plan(plan_with_joints)


    def go_to_pregrasp_position (self, object_pose, pregrasp_distance=0.5):
        """ It should be already a box added to the scene"""
          
        if type (object_pose) is not type (geometry_msgs.msg.PoseStamped()): 
            rospy.logerr ( "The object pose should be a geometry_msgs.msg.PoseStamped()")
            return 

        else:  
            pregrasp_pose  = copy.deepcopy(object_pose)
            pregrasp_pose.pose.position.z += pregrasp_distance #Go to 50 cm above the box 
            pregrasp_pose.pose.orientation.x = 0
            pregrasp_pose.pose.orientation.y= 1
            pregrasp_pose.pose.orientation.z = 0
            pregrasp_pose.pose.orientation.w =0
            plan_to_pregraps= self.ur10_commander.plan_to_pose_target (pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y, pregrasp_pose.pose.position.z, pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y, pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w)    

            if len(plan_to_pregraps.joint_trajectory.points)>0: 
                print "Plan to pregrasp"
                raw_input("if Plan is ok. press enter to execute")

            else: 
                rospy.logerr ( "Fail to make pregrasp plan")

            self.ur10_commander.execute_plan(plan_to_pregraps)

    def go_to_grasp_position(self, grasp_distance=0.21):
        """ It should be already a box added to the scene"""
        
        waypoints = []
            
        grasp_pose= self.ur10_commander.group.get_current_pose().pose
        grasp_pose.position.x = self.position1.pose.position.x
        grasp_pose.position.y = self.position1.pose.position.y
        grasp_pose.position.z = self.position1.pose.position.z + grasp_distance
        waypoints.append(copy.deepcopy(grasp_pose))
        (plan_to_grasp, fraction) = self.ur10_commander.group.compute_cartesian_path (waypoints,0.01,0.0)

        if len(plan_to_grasp.joint_trajectory.points)>0: 
                print "Plan to grasp"
                print "Fraction =", fraction
                raw_input("if Plan is ok. press enter to execute")

        else: 
            rospy.logerr ( "Fail to make pregrasp plan")

        self.ur10_commander.execute_plan(plan_to_grasp)
        
        

        


# # 1. Initialize
# #=========================================================================================================

if __name__ == '__main__':

    rospy.init_node("move_commander_ur10", anonymous= True) # Define the node
    position1= geometry_msgs.msg.PoseStamped() 
    position1.pose.position.x = 0.92
    position1.pose.position.y = 0.36
    position1.pose.position.z = 0.25
    position1.pose.orientation.w = 1
   # print position1
    
    position2= geometry_msgs.msg.PoseStamped()
    pick_and_place1 = PickAndPlace(position1, position2)



# # 2. Go to initial position (Position 1)
# # ========================================================================================================
# #!!!!Change the values to pos 1 


# # 3. Add box in position 2 
# #=========================================================================================================

# ur10_commander.add_box(0.92,0.36,0.25)
    
# time.sleep (5)  

# # 4. Go to pregrasp position (30 cm above the box)
# # ========================================================================================================
# #NOT working 
# def go_to_pregrasp_position (object_pose, pregrasp_distance=0.3):
#     """ It should be already a box added to the scene"""
        
#     if type (object_pose) is not geometry_msgs.msg.PoseStamped(): 
#         rospy.logerr ( "The object pose should be a geometry_msgs.msg.PoseStamped()")
#         return 

#     else:  
#         wpose = object_pose
#         wpose.position_z += 0.3 #Go to 30 cm above the box 
#         ur10_commander.plan_to_pose_target (object_pose.position.x, object_pose.position.y, object_pose.position.z, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)    

# plan_to_pregraps= go_to_pregrasp_position(ur10_commander, self.box_pose, pregrasp_distance=0.3)

# if len(plan_to_pos3.joint_trajectory.points)>0: 
#     print "Plan to 1"
#     raw_input("if Plan is ok. press enter to execute")

# else: 
#     rospy.logerr ( "Fail to make a plan")

# ur10_commander.execute_plan(plan_to_pregraps)

# # 5. Go to grasp position (1 cm above the box)
# # ========================================================================================================
#    # THIS FUNCTION SHOULD BE ALSO IN PICK AND PLACE WITH THE SAME FORMAT THAT THE ONE BEFORE...
#     # def go_to_grasp_position (self, grasp_distance):
#     #     """ It should be already a box added to the scene"""
        
#     #     if self.box_name == "": 
#     #         rospy.logerr ( "There is no box in the scene ")
#     #         return 
  
#     #     else:  
#     #         waypoints = []
            
#     #         # wpose = self.box_pose
#     #         # wpose.position_z += 0.3 #Go to 30 cm above the box 
#     #         # waypoints.append(copy.deepcopy(wpose))

#     #         wpose = copy.deepcopy(self.box_pose)
#     #         wpose.position_z += 0.01 #Go to 1 cm above the box 
#     #         waypoints.append(copy.deepcopy(wpose))

#     #         (plan, fraction) = self.group.compute_cartesian_path (waypoints,0.01,0.0)
            
#     #         return plan, fraction 


# # 6. Attach the box
# #=========================================================================================================

# ur10_commander.attach_box()

# time.sleep (5) 

# # 7. Move the box to position 3. 
# # ========================================================================================================
# #!!!Change the values to pos3

# plan_to_pos3 = ur10_commander.plan_to_pose_target(-1.2768, 0.36727, 0.26319,-0.013261,0.70689,0.70708 ,0.012899)

# if len(plan_to_pos3.joint_trajectory.points)>0: 
#     print "Plan to 1"
#     raw_input("if Plan is ok. press enter to execute")

# else: 
#     rospy.logerr ( "Fail to make a plan")

# ur10_commander.execute_plan(plan_to_pos3)


# # 8. Detach the box 
# # ========================================================================================================

# ur10_commander.detach_box()
# time.sleep (5) 

# # 9. Go to vertical position 
# # ======================================================================================================== 
# plan_to_k= ur10_commander.plan_to_pose_target(0.04016, 0.36714, 1.52268,0.00021975, 0.70679, 0.70742, -0.00028481)
     
# if len(plan_to_k.joint_trajectory.points)>0: 
#     print "Plan to i"
#     raw_input("if Plan is ok. press enter to execute")

# else: 
#     rospy.logerr ( "Fail to make a plan")

# ur10_commander.execute_plan(plan_to_k)

# # 10. Delete the box
# # =========================================================================================================

# ur10_commander.remove_box()
   
