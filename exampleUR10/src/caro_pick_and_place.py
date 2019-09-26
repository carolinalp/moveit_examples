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
            self.position_pick = self.get_object_pose_stamped(object_name)
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
        
    def get_object_pose_stamped(self,object_name):
        pose_object = self.ur10_commander.get_object_pose(object_name)[object_name] # ur10_commander_ex1.get_object_pose("box")["box"]
        pose_stamped = self.ur10_commander.pose_stamped(pose_object)
        
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
    position1.pose.position.x = 0.9
    position1.pose.position.y = 0.3
    position1.pose.position.z = 0.82
    position1.pose.orientation.w = 1
    # Position 2
    position2 = geometry_msgs.msg.PoseStamped()
    position2.pose.position.x = 0.6
    position2.pose.position.y = 0.3 
    position2.pose.position.z = 0.85
    position2.pose.orientation.w = 1
    
    
    ur10_commander_ex1=MoveCommanderUr10()
    time.sleep(0.5)   
    dict_add_objects={"box": position1, "mesh":position2}
    filename_mesh= "/home/user/workspace/src/moveit_examples/exampleUR10/models/coffee_mug_120602a_ctr.stl"
    #Add objects
    for key in dict_add_objects:
        position= dict_add_objects[key]
        ur10_commander_ex1.add_object(key, position.pose.position.x,
                                    position.pose.position.y,
                                    position.pose.position.z,filename_mesh=filename_mesh)
        time.sleep(1)
    #Move objects
    print  ur10_commander_ex1.get_known_object_names()
    list_move_objects=['box','mesh']
    #list_move_objects=['box','orange__link'] ###NOT possible Which function should I use to attach the object?
    for object in list_move_objects:
        pose_pick= copy.deepcopy(ur10_commander_ex1.get_object_pose(object)[object])       
        position_place= ur10_commander_ex1.pose_stamped(pose_pick)
        position_place.pose.position.y = position_place.pose.position.y + 0.2
        PickAndPlace_ex1  = PickAndPlace(object, position_place)
        time.sleep(1)
    
    
    #Remove objects
    for object in list_move_objects:        
        ur10_commander_ex1.remove_object(object)
        time.sleep(1)
