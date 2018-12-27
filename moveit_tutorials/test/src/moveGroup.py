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
		scene = moveit_commander.PlanningSceneInterface()
		# scene = moveit_python.planning_scene_interface.PlanningSceneInterface("base_link")
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

		current_pose = group.get_current_pose()
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

	def go_to_pose_goal(self):

		group = self.group

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.x = 0.707105482511
		pose_goal.orientation.y = 0
		pose_goal.orientation.z = 0
		pose_goal.orientation.w = 0.707105482511
		pose_goal.position.x = 0.2
		pose_goal.position.y = 0
		pose_goal.position.z = 0.06
		group.set_pose_target(pose_goal)

		## Now, we call the planner to compute the plan and execute it.
		plan = group.go(wait=True)

		# plan = group.plan(pose_goal)

		# group.execute(plan, wait=True)
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()

		print "current Pose"
		print group.get_current_pose()

		print "current Joint"
		print group.get_current_joint_values()


def main():
	# print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
	# raw_input()
	tutorial = MoveGroupPythonIntefaceTutorial()

	# tutorial.add_box()
	tutorial.reset_to_zero_state()

	tutorial.go_to_pose_goal()

	# while( tutorial.poseCnt < 448 ):
	# 	tutorial.reset_to_zero_state()
	# 	tutorial.go_to_random_goal()



	print tutorial.poseList
	print tutorial.jointList
	with open('./poseList.josn','w') as poseFile:
		
		data = json.dumps(tutorial.poseList)
		poseFile.write(data)

	with open('./jointList.josn','w') as jointFile:
		
		data = json.dumps(tutorial.jointList)
		jointFile.write(data)

	print "============ Python tutorial demo complete!"


if __name__ == '__main__':
	main()









