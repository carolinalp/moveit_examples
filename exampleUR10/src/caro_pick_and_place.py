#!/usr/bin/env python
import copy
import geometry_msgs.msg
import rospy
import time
from caro_moveit_commander import MoveCommanderUr10


class PickAndPlace (object):
    """ In this class, the robot picks an object in the scence, moves the robot to home,
    then grasp the object and move it to position_place
    then detach the object, returns to home.
    Assumes that object is already created in the scene
    Position_place is an element of the type geometry_msgs.msg.PoseStamped """

    def __init__(self, object_name, position_place):

        self.ur10_commander = MoveCommanderUr10()       
        self.object_name= object_name        
        self.position_place = position_place
        self.position_pick = geometry_msgs.msg.PoseStamped()

        if self.ur10_commander.is_an_object(self.object_name):
            self.position_pick = self.pose_stamped(object_name)
            self.go_to_home_position()
            self.go_to_pregrasp_position(self.position_pick)
            time.sleep(0.5)
            self.go_to_grasp_position(self.position_pick)
            time.sleep(0.5)
            self.ur10_commander.attach_object(self.object_name)
            time.sleep(0.5)
            self.move_object(self.position_place)
            time.sleep(0.5)
            self.ur10_commander.detach_object()
            time.sleep(0.5)
            self.go_to_home_position()
            time.sleep(0.5)

        else:
            rospy.logerr("There is no object called " + object_name + " in the scene")                            
        
    def pose_stamped(self,object_name):
        pose_object = self.ur10_commander.get_object_pose(object_name)[object_name] # ur10_commander_ex1.get_object_pose("box")["box"]
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose.position.x = pose_object.position.x
        pose_stamped.pose.position.y = pose_object.position.y
        pose_stamped.pose.position.z = pose_object.position.z
        pose_stamped.pose.orientation.x = pose_object.orientation.x
        pose_stamped.pose.orientation.y = pose_object.orientation.y
        pose_stamped.pose.orientation.z = pose_object.orientation.z
        pose_stamped.pose.orientation.w = pose_object.orientation.w

        return pose_stamped

    def go_to_home_position(self):

        plan_with_joints = self.ur10_commander.go_to_joint_target(-0.03883398363815905, -1.5276687999544085, 1.5388371215293597, 3.1372948604019957, 0.039511801410542426, 3.134863975410523)
        if len(plan_with_joints.joint_trajectory.points) == 0:
            
            rospy.logerr("Fail to make a plan")
            
        rospy.loginfo("Going home")
        self.ur10_commander.execute_plan(plan_with_joints)

    def go_to_pregrasp_position(self, object_pose, pregrasp_distance=0.5):
        """ It should be already a box added to the scene"""
          
        if not isinstance(object_pose, geometry_msgs.msg.PoseStamped):
            rospy.logerr("The object pose should be a geometry_msgs.msg.PoseStamped()")
            return
 
        pregrasp_pose = copy.deepcopy(object_pose)
        pregrasp_pose.pose.position.z += pregrasp_distance
        pregrasp_pose.pose.orientation.x = 0
        pregrasp_pose.pose.orientation.y = 1
        pregrasp_pose.pose.orientation.z = 0
        pregrasp_pose.pose.orientation.w = 0
        plan_to_pregraps = self.ur10_commander.plan_to_pose_target(pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y, pregrasp_pose.pose.position.z, pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y, pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w)    

        if len(plan_to_pregraps.joint_trajectory.points) > 0:
            rospy.loginfo("Going to Pregrasp position")
            
        else:
            rospy.logerr("Fail to make pregrasp plan")

        self.ur10_commander.execute_plan(plan_to_pregraps)

    def go_to_grasp_position(self, object_pose, grasp_distance=0.21):
        """ It should be already a box added to the scene"""

        if not isinstance(object_pose, geometry_msgs.msg.PoseStamped):
            rospy.logerr("The object pose should be a geometry_msgs.msg.PoseStamped()")
            return

        waypoints = []
        
        grasp_pose = self.ur10_commander.group.get_current_pose().pose
        grasp_pose.position.x = object_pose.pose.position.x
        grasp_pose.position.y = object_pose.pose.position.y
        grasp_pose.position.z = object_pose.pose.position.z + grasp_distance
        waypoints.append(copy.deepcopy(grasp_pose))
        (plan_to_grasp, fraction) = self.ur10_commander.group.compute_cartesian_path(waypoints, 0.01, 0.0)

        if len(plan_to_grasp.joint_trajectory.points) > 0:
            rospy.loginfo("Going to Grasp position")
            rospy.loginfo("Fraction = %s" % fraction)
        else:
            rospy.logerr("Fail to make grasp plan")

        self.ur10_commander.execute_plan(plan_to_grasp)
        
    def move_object(self, place_pose, grasp_distance=0.21):
        
        # Test if place_pose is a poseStamped
        if not isinstance(place_pose, geometry_msgs.msg.PoseStamped):
            rospy.logerr("The object pose should be a geometry_msgs.msg.PoseStamped()")
            return

        # Test if the box is attached
        attached_objects = self.ur10_commander.scene_interface.get_attached_objects()
        is_attached = len(attached_objects.keys()) > 0
        if is_attached:
            final_pose = copy.deepcopy(place_pose)
            final_pose.pose.position.z += grasp_distance
            final_pose.pose.orientation.x = 0
            final_pose.pose.orientation.y = 1
            final_pose.pose.orientation.z = 0
            final_pose.pose.orientation.w = 0

            plan_to_final_pose = self.ur10_commander.plan_to_pose_target(final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z, final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w)    

            if len(plan_to_final_pose.joint_trajectory.points) > 0:
                rospy.loginfo("Moving the box")
            else:
                rospy.logerr("Fail to make plan to pose")

            self.ur10_commander.execute_plan(plan_to_final_pose)

        else:
            rospy.logerr("There is no object attached")


if __name__ == '__main__':  
    # Define the node
    rospy.init_node("move_commander_ur10", anonymous=True)
    # Position 1
    position1 = geometry_msgs.msg.PoseStamped() 
    position1.pose.position.x = 0.92
    position1.pose.position.y = 0.5
    position1.pose.position.z = 0.25
    position1.pose.orientation.w = 1
    # Position 2
    position2 = geometry_msgs.msg.PoseStamped()
    position2.pose.position.x = position1.pose.position.x - 0.4
    position2.pose.position.y = position1.pose.position.y 
    position2.pose.position.z = position1.pose.position.z
    position2.pose.orientation.w = 1
    # Position 3
    position3 = geometry_msgs.msg.PoseStamped()
    position3.pose.position.x = position1.pose.position.x 
    position3.pose.position.y = position1.pose.position.y - 0.8
    position3.pose.position.z = position1.pose.position.z
    position3.pose.orientation.w = 1
    #Add objects
    ur10_commander_ex1=MoveCommanderUr10()
    time.sleep(0.5)   
  #  ur10_commander_ex1.add_object("box", position1.pose.position.x,
                            #        position1.pose.position.y,
                                #    position1.pose.position.z)
  #  time.sleep(0.5)    
  #  ur10_commander_ex1.add_object("mesh", position2.pose.position.x,
           #                         position2.pose.position.y,
             #                       position2.pose.position.z, filename_mesh= "/home/user/workspace/src/moveit_examples/exampleUR10/models/coffee_mug_120602a_ctr.stl")
  #  time.sleep(0.5) 
    #print ur10_commander_ex1.is_an_object("mesh")
    
    #print ur10_commander_ex1.is_an_object("box")
    #print ur10_commander_ex1.get_object_pose("box")["box"]
    #Pick and Place
   # PickAndPlaceBox= PickAndPlace("box", position3) 
    # time.sleep(0.5) 
   # PickAndPlaceMesh  = PickAndPlace("mesh", position1)
   # time.sleep(0.5)
   # ur10_commander_ex1.remove_object("mesh")
   # time.sleep(0.5)
   # ur10_commander_ex1.remove_object("box")

    dict_object={"box": position1, "mesh":position2}
    filename_mesh= "/home/user/workspace/src/moveit_examples/exampleUR10/models/coffee_mug_120602a_ctr.stl"
    for key in dict_object:
        position=dict_object[key]
        ur10_commander_ex1.add_object(key, position.pose.position.x,
                                    position.pose.position.y,
                                    position.pose.position.z,filename_mesh=filename_mesh)
        #position_place=position
        #position_place.pose.position.y -= 0.8
       
       # PickAndPlaceMesh  = PickAndPlace(key, position_place)
        #time.sleep(0.5)
       # ur10_commander_ex1.remove_object(key)
        time.sleep(1)
    
    for key in dict_object:
        position=dict_object[key]
        position_place=position
        position_place.pose.position.y -= 0.8        
        PickAndPlaceMesh  = PickAndPlace(key, position_place)
        time.sleep(1)
       # ur10_commander_ex1.remove_object(key)
        time.sleep(0.5)

    for key in dict_object:        
        ur10_commander_ex1.remove_object(key)
        time.sleep(1)
