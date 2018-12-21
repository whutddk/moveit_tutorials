#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import json
import moveit_python

pi = 3.14159265

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class MoveGroupPythonIntefaceTutorial(object):
	"""MoveGroupPythonIntefaceTutorial"""
	def __init__(self):
		super(MoveGroupPythonIntefaceTutorial, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface_tutorial',
						anonymous=True)
		robot = moveit_commander.RobotCommander()
		# scene = moveit_commander.PlanningSceneInterface()
		scene = moveit_python.planning_scene_interface.PlanningSceneInterface("base_link")
		group_name = "anno_arm"
		group = moveit_commander.MoveGroupCommander(group_name)
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
													   moveit_msgs.msg.DisplayTrajectory,
													   queue_size=20)
		planning_frame = group.get_planning_frame()
		print "============ Reference frame: %s" % planning_frame
		eef_link = group.get_end_effector_link()
		print "============ End effector: %s" % eef_link
		group_names = robot.get_group_names()
		print "============ Robot Groups:", robot.get_group_names()
		print "============ Printing robot state"
		print robot.get_current_state()
		print ""
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

		self.poseList = []
		self.jointList = []
		self.poseCnt = 0;


	def reset_to_zero_state(self):
		group = self.group
		joint_goal = group.get_current_joint_values()

		joint_goal[0] = 0
		joint_goal[1] = 0
		joint_goal[2] = 0
		joint_goal[3] = 0
		joint_goal[4] = 0
		joint_goal[5] = 0

		# plan = group.plan(joint_goal)
		# group.execute(plan, wait=True)
		plan = group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		group.stop()

		if (plan == False):
			sys.exit(-1)
		current_joints = self.group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)


	def go_to_random_goal(self):
		group = self.group
		pose_goal = geometry_msgs.msg.Pose()

		# get_random_pose

		randomPose = group.get_random_pose()
		group.set_pose_target(randomPose)
		# group.set_pose_target([0.2,-0.2,0.05,1.57,0,1.57])
		plan = group.go(wait=True)

		print plan
		group.stop()
		group.clear_pose_targets()

		current_pose = self.group.get_current_pose()
		joint_goal = group.get_current_joint_values()

		if ( plan ):
			

			if ( (joint_goal[0] < pi) and (joint_goal[0] > -pi) 
				and (joint_goal[1] < pi*115/180) and (joint_goal[1] > -pi*115/180) 
				and (joint_goal[2] < pi*220/180) and (joint_goal[2] > -pi*40/180)
				and (joint_goal[3] < pi) and (joint_goal[3] > -pi)
				and (joint_goal[4] < pi*225/180) and (joint_goal[4] > -pi*45/180)
				and (joint_goal[5] < pi) and (joint_goal[5] > -pi)
				and current_pose.pose.position.z > 0):

				poseList = [current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w]
				print poseList
				print joint_goal
				self.poseList.append(poseList)
				self.jointList.append(joint_goal)
				self.poseCnt = self.poseCnt + 1;
				print self.poseCnt


		current_pose = current_pose.pose
		return all_close(pose_goal, current_pose, 0.01)

	# def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
	# 	# Copy class variables to local variables to make the web tutorials more clear.
	# 	# In practice, you should use the class variables directly unless you have a good
	# 	# reason not to.
	# 	box_name = self.box_name
	# 	scene = self.scene

	# 	## BEGIN_SUB_TUTORIAL wait_for_scene_update
	# 	##
	# 	## Ensuring Collision Updates Are Receieved
	# 	## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	# 	## If the Python node dies before publishing a collision object update message, the message
	# 	## could get lost and the box will not appear. To ensure that the updates are
	# 	## made, we wait until we see the changes reflected in the
	# 	## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
	# 	## For the purpose of this tutorial, we call this function after adding,
	# 	## removing, attaching or detaching an object in the planning scene. We then wait
	# 	## until the updates have been made or ``timeout`` seconds have passed
	# 	start = rospy.get_time()
	# 	seconds = rospy.get_time()
	# 	while (seconds - start < timeout) and not rospy.is_shutdown():
	# 		# Test if the box is in attached objects
	# 		# attached_objects = scene.get_attached_objects([box_name])
	# 		attached_objects = scene.getKnowAttachedObjects([box_name])
	# 		is_attached = len(attached_objects.keys()) > 0

	# 		# Test if the box is in the scene.
	# 		# Note that attaching the box will remove it from known_objects
	# 		is_known = box_name in scene.get_known_object_names()

	# 		# Test if we are in the expected state
	# 		if (box_is_attached == is_attached) and (box_is_known == is_known):
	# 			return True

	# 		# Sleep so that we give other threads time on the processor
	# 		rospy.sleep(0.1)
	# 		seconds = rospy.get_time()

	# 	# If we exited the while loop without returning then we timed out
	# 	return False
	# 	## END_SUB_TUTORIAL

	# def add_box(self, timeout=4):
	# 	# Copy class variables to local variables to make the web tutorials more clear.
	# 	# In practice, you should use the class variables directly unless you have a good
	# 	# reason not to.
	# 	box_name = self.box_name
	# 	scene = self.scene

	# 	## BEGIN_SUB_TUTORIAL add_box
	# 	##
	# 	## Adding Objects to the Planning Scene
	# 	## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	# 	## First, we will create a box in the planning scene at the location of the left finger:
	# 	# box_pose = geometry_msgs.msg.PoseStamped()
	# 	# box_pose.header.frame_id = "base_footprint"
	# 	# box_pose.pose.orientation.w = 1.0

	# 	# box_pose.pose.position.x = 0.3
	# 	# box_pose.pose.position.y = 0
	# 	# box_pose.pose.position.z = 0.3

	# 	box_name = "box-0-0-0"
	# 	scene.addBox(box_name, 0.4,0.4,0.4, 0, 0, 0)
	# 	scene.addBox("box-0-0-1", 0.4,0.4,0.4, 0, 0.2, 0)

	# 	self.box_name=box_name
	# 	return 

	# def remove_box(self, timeout=4):
	# 	# Copy class variables to local variables to make the web tutorials more clear.
	# 	# In practice, you should use the class variables directly unless you have a good
	# 	# reason not to.
	# 	box_name = self.box_name
	# 	scene = self.scene

	# 	# scene.removeCollisionObject(box_name)
	# 	scene.clear()
	# 	return 

	def collision_detect(self):
		scene = self.scene
		print scene.getKnownCollisionObjects()


def main():
	try:
		# print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
		# raw_input()
		tutorial = MoveGroupPythonIntefaceTutorial()

		# print "============ Press `Enter` to ..."
		# raw_input()
		# tutorial.add_box()
		# tutorial.collision_detect()

		# print "============ Press `Enter` to ..."
		# raw_input()

		while( tutorial.poseCnt < 448 ):
			tutorial.reset_to_zero_state()
			tutorial.go_to_random_goal()

		# print "============ Press `Enter` to a..."
		# raw_input()
		
		# tutorial.add_box()

		# print "============ Press `Enter` to ..."
		# raw_input()
		# # tutorial.remove_box()
		# tutorial.go_to_joint_state(pi/2,-pi/2,0,0,pi/2,0)

		# print "============ Press `Enter` to ..."
		# raw_input()
		# tutorial.remove_box()
		print tutorial.poseList
		print tutorial.jointList
		with open('./poseList.josn','w') as poseFile:
			
			data = json.dumps(tutorial.poseList)
			poseFile.write(data)

		with open('./jointList.josn','w') as jointFile:
			
			data = json.dumps(tutorial.jointList)
			jointFile.write(data)

		print "============ Python tutorial demo complete!"
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()









