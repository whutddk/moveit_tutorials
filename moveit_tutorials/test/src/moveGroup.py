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




edgeIndexFile = []
poseList = []
jointList = []
edgeNum = 0

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

	global poseList
	global jointList
	global edgeIndex

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


	def go_to_mid_state(self):
		group = self.group
		joint_goal = group.get_current_joint_values()

		joint_goal[0] = 0
		joint_goal[1] = 0
		joint_goal[2] = pi/2
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

		# print plan
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
				and current_pose.pose.position.z > 0.6):

				poseListTemp = [current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w]
				
				# print poseListTemp
				# print joint_goal

				poseList.append(poseListTemp)
				jointList.append(joint_goal)
				
		current_pose = current_pose.pose
		all_close(pose_goal, current_pose, 0.01)

		return plan

	def go_to_pose_goal(self,X,Y,Z):

		group = self.group

		group.set_pose_target([X,Y,Z,1.57,0,1.57])

		## Now, we call the planner to compute the plan and execute it.
		plan = group.go(wait=True)

		group.stop()

		current_pose = group.get_current_pose()

		joint_goal = group.get_current_joint_values()

		poseListTemp = [current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w]
		
		poseList.append(poseListTemp)
		jointList.append(joint_goal)

		group.clear_pose_targets()




def load_poseList():
	global poseList

	with open('./poseList.json','r') as poseListFile:
			
		data = poseListFile.read()
		poseList = json.loads(data)

def save_poseList():
	global poseList
	with open('./poseList.json','w') as poseListFile:
		
		data = json.dumps(poseList)
		poseListFile.write(data)


def load_jointList():
	global jointList

	with open('./jointList.json','r') as jointListFile:
			
		data = jointListFile.read()
		jointList = json.loads(data)

def save_jointList():
	global jointList
	with open('./jointList.json','w') as jointListFile:
		
		data = json.dumps(jointList)
		jointListFile.write(data)



def load_edgeIndex():
	global edgeIndex

	with open('./edgeIndex.json','r') as edgeIndexFile:
			
		data = edgeIndexFile.read()
		edgeIndex = json.loads(data)	

def save_edgeIndex():
	global edgeIndex
	with open('./edgeIndex.json','w') as edgeIndexFile:
		
		data = json.dumps(edgeIndex)
		edgeIndexFile.write(data)

def edge_constraint():
	global jointList
	global edgeIndex
	global edgeNum

	newIndex = len(poseList) - 1

	for preIndex in range(0,newIndex):
	# for prePose in poseList:
		if ( ( abs(jointList[preIndex][0] - jointList[newIndex][0]) < (0.417 / 180 * pi) ) and
			( abs(jointList[preIndex][1] - jointList[newIndex][1]) < (0.183 / 180 * pi) ) and
			( abs(jointList[preIndex][2] - jointList[newIndex][2]) < (0.25 / 180 * pi) ) and
			( abs(jointList[preIndex][3] - jointList[newIndex][3]) < (0.2 / 180 * pi) ) and
			( abs(jointList[preIndex][4] - jointList[newIndex][4]) < (0.2 / 180 * pi) ) and
			( abs(jointList[preIndex][5] - jointList[newIndex][5]) < (0.543 / 180 * pi) ) ):
			edge = [preIndex,newIndex]
			edgeIndex.append(edge)
			edgeNum = edgeNum + 1

			print edge
			print len(jointList)



def main():

	global poseList
	global jointList
	global edgeIndex
	global edgeNum


	load_poseList()
	load_jointList()
	load_edgeIndex()
	
	tutorial = MoveGroupPythonIntefaceTutorial()
	tutorial.reset_to_zero_state()

	tutorial.go_to_mid_state()
	current_pose = tutorial.group.get_current_pose()
	print "current Pose"
	print current_pose

	joint_goal = tutorial.group.get_current_joint_values()
	print "current Joint"
	print joint_goal

	poseListTemp = [current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w]
	
	poseList.append(poseListTemp)
	jointList.append(joint_goal)



	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.18,0.15,0.06)
	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.18,0.0,0.06)
	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.18,-0.15,0.06)

	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.24,0.15,0.06)
	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.24,0.0,0.06)
	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.24,-0.15,0.06)

	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.34,0.15,0.06)
	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.34,0.0,0.06)
	tutorial.reset_to_zero_state()
	tutorial.go_to_pose_goal(0.34,-0.15,0.06)

	save_jointList()
	save_poseList()

	edgeNum = len(edgeIndex)
	print edgeNum

	while( edgeNum < 100000 ):
		tutorial.reset_to_zero_state()
		if ( True == tutorial.go_to_random_goal()):
			edge_constraint()
			save_edgeIndex()

		save_jointList()
		save_poseList()


	print poseList
	print jointList




	print "============ Python tutorial demo complete!"


if __name__ == '__main__':
	main()









