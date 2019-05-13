#!/usr/bin/env python
import copy
import geometry_msgs.msg
import rospy
import time
from caro_moveit_commander import MoveCommanderUr10


class PickAndPlace (object):
    """ This class add a box in position1, moves the robot to home, then to
    pregrasp and graps positions. Grasp the box and move it to the position2,
    then detach the box, returns to home and finally removes the box.
    Assumes that position1 and position2 are elements of the type
    geometry_msgs.msg.PoseStamped """

    def __init__(self, position1, position2):

        self.ur10_commander = MoveCommanderUr10()
        self.position1 = position1
        self.position2 = position2
        time.sleep(2)
        self.ur10_commander.add_box(self.position1.pose.position.x, self.position1.pose.position.y, self.position1.pose.position.z)
        time.sleep(2)
        self.go_to_home_position()         
        time.sleep(2)
        self.go_to_pregrasp_position(self.position1)
        time.sleep(2)
        self.go_to_grasp_position(self.position1)
        time.sleep(2)
        self.ur10_commander.attach_box()
        time.sleep(2)
        self.move_box(self.position2)
        time.sleep(2)
        self.ur10_commander.detach_box()
        time.sleep(2)
        self.go_to_home_position()
        time.sleep(2)
        self.ur10_commander.remove_box()
        
    def go_to_home_position(self):

        plan_with_joints = self.ur10_commander.go_to_joint_target(-0.03883398363815905, -1.5276687999544085, 1.5388371215293597, 3.1372948604019957, 0.039511801410542426, 3.134863975410523)
    
        if len(plan_with_joints.joint_trajectory.points) == 0:
            
            rospy.logerr("Fail to make a plan") 
            
        print "Plan to home "
        self.ur10_commander.execute_plan(plan_with_joints)

    def go_to_pregrasp_position(self, object_pose, pregrasp_distance=0.5):
        """ It should be already a box added to the scene"""
          
        if type(object_pose) is not type(geometry_msgs.msg.PoseStamped()):
            rospy.logerr("The object pose should be a geometry_msgs.msg.PoseStamped()")
            return

        else:
            pregrasp_pose = copy.deepcopy(object_pose)
            pregrasp_pose.pose.position.z += pregrasp_distance
            pregrasp_pose.pose.orientation.x = 0
            pregrasp_pose.pose.orientation.y = 1
            pregrasp_pose.pose.orientation.z = 0
            pregrasp_pose.pose.orientation.w = 0
            plan_to_pregraps = self.ur10_commander.plan_to_pose_target(pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y, pregrasp_pose.pose.position.z, pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y, pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w)    

            if len(plan_to_pregraps.joint_trajectory.points) > 0:
                print "Plan to pregrasp"
               
            else: 
                rospy.logerr("Fail to make pregrasp plan")

            self.ur10_commander.execute_plan(plan_to_pregraps)

    def go_to_grasp_position(self, object_pose, grasp_distance=0.21):
        """ It should be already a box added to the scene"""

        waypoints = []
        
        grasp_pose = self.ur10_commander.group.get_current_pose().pose
        grasp_pose.position.x = object_pose.pose.position.x
        grasp_pose.position.y = object_pose.pose.position.y
        grasp_pose.position.z = object_pose.pose.position.z + grasp_distance
        waypoints.append(copy.deepcopy(grasp_pose))
        (plan_to_grasp, fraction) = self.ur10_commander.group.compute_cartesian_path(waypoints, 0.01, 0.0)

        if len(plan_to_grasp.joint_trajectory.points) > 0:
            print "Plan to grasp"
            print "Fraction =", fraction
        else: 
            rospy.logerr("Fail to make grasp plan")

        self.ur10_commander.execute_plan(plan_to_grasp)
        
    def move_box(self, place_pose, grasp_distance=0.21):
        
        if type(place_pose) is not type(geometry_msgs.msg.PoseStamped()): 
            rospy.logerr("The object pose should be a geometry_msgs.msg.PoseStamped()")
            return

        else:
            # Test if the box is attached

            attached_objects = self.ur10_commander.scene_interface.get_attached_objects()
            is_attached = len(attached_objects.keys()) > 0
        
            if is_attached is True:
                
                final_pose = copy.deepcopy(place_pose)
                final_pose.pose.position.z += grasp_distance
                final_pose.pose.orientation.x = 0
                final_pose.pose.orientation.y = 1
                final_pose.pose.orientation.z = 0
                final_pose.pose.orientation.w = 0

                plan_to_final_pose = self.ur10_commander.plan_to_pose_target(final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z, final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w)    

                if len(plan_to_final_pose.joint_trajectory.points) > 0:
                    print "Plan to place"
                else: 
                    rospy.logerr("Fail to make plan to pose")

                self.ur10_commander.execute_plan(plan_to_final_pose)

# # 1. Initialize
# #=========================================================================================================

if __name__ == '__main__':
    
    # Define the node
    rospy.init_node("move_commander_ur10", anonymous=True)
    # Position 1
    position1 = geometry_msgs.msg.PoseStamped() 
    position1.pose.position.x = 0.92
    position1.pose.position.y = 0.36
    position1.pose.position.z = 0.25
    position1.pose.orientation.w = 1
    # Position 2
    position2 = geometry_msgs.msg.PoseStamped()
    position2.pose.position.x = 0.92
    position2.pose.position.y = -0.5
    position2.pose.position.z = 0.25
    position2.pose.orientation.w = 1
    pick_and_place1 = PickAndPlace(position1, position2)