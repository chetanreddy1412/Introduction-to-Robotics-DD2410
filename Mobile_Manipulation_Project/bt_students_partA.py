#!/usr/bin/env python

# group members: Chetan Reddy Narayanaswamy, Florentina Voboril

import numpy as np
import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_srvs.srv import SetBool, Empty
import actionlib
import move_base_msgs.msg as mb_msgs
from actionlib_msgs.msg import GoalStatus

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

num_of_resets = 0

class BehaviourTree(ptr.trees.BehaviourTree):
	def __init__(self):
		rospy.loginfo("######### Initialising behaviour tree ##########")

		b_move = pt.composites.Selector(
			name="Go back",
			children=[counter_in_loop(5, "Has gone back?"), go("Go back!", -0.7, 0)]
		)

		b_move_2 = pt.composites.Selector(
			name="Go back 2",
			children=[counter_in_loop(10, "Has gone back 2?"), go("Go back 2!", -0.7, 0)]
		)

		b_cov = CheckCovariance()
		b_loc = Localization()
		b_check_and_loc = pt.composites.Selector(
			name='Localise if High Covariance',
			children=[b_cov,b_loc]
		)
		tree = RSequence(name="Main sequence", children = [movehead("up"), b_move_2, tuckarm(), b_check_and_loc, Navigation("pick"), movehead("down"), PickCube(), movehead("up"), b_move, Navigation("place"), movehead("down"), PlaceCube(), Check()])
		super(BehaviourTree, self).__init__(tree)


		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)



class CheckCovariance(pt.behaviour.Behaviour):
	def __init__(self,t1=0.03,t2=0.04):
		rospy.loginfo("Initialising CheckCovariance")

		#Randomise Particles Service
		self.randomise_particles_srv_name = rospy.get_param(rospy.get_name() + '/global_loc_srv') #from the amcl documentation
		rospy.wait_for_service(self.randomise_particles_srv_name, timeout = 20)
		self.randomise_particles_srv = rospy.ServiceProxy(self.randomise_particles_srv_name, Empty)

		self.randomise_particles_srv()

		#Clear Costmap Service
		self.clear_costmaps_srv_name = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
		rospy.wait_for_service(self.clear_costmaps_srv_name, timeout = 20)
		self.clear_costmaps_srv = rospy.ServiceProxy(self.clear_costmaps_srv_name, Empty)

		self.t1 = t1
		self.t2 = t2


		self.amcl_pose_topic_nm = rospy.get_param(rospy.get_name()+'/amcl_estimate')
		self.amcl_cov = None
		rospy.Subscriber(self.amcl_pose_topic_nm, PoseWithCovarianceStamped,self.callback_pose)

		rospy.sleep(2)

		if self.amcl_cov == None:
			raise Exception(' Did not subscribe to amcl_pose')

		self.position_known = False
		super(CheckCovariance, self).__init__("Check Covariance!")

	def callback_pose(self,data):
		#print('Bro i was called')
		if data!=None:
			self.amcl_cov = np.linalg.norm(data.pose.covariance)
			#print('Cov: ',self.amcl_cov)

	def update(self):

		# print('Covariance',round(self.amcl_cov,5),'Position Known Flag: ',self.position_known)
		if self.position_known:
			if self.amcl_cov < self.t2:
				
				return pt.common.Status.SUCCESS
			else:
				self.position_known = False
				
				self.randomise_particles_srv()
				self.clear_costmaps_srv()
				return pt.common.Status.FAILURE

		if self.position_known == False:
			self.clear_costmaps_srv()
			if self.amcl_cov<self.t1:
				self.position_known = True
				return pt.common.Status.SUCCESS
			
			else:
				return pt.common.Status.FAILURE
			



		'''
		if self.amcl_cov<self.threshold:
			print('Covariance is low enough: ',round(self.amcl_cov,5))
			return pt.common.Status.SUCCESS

		else:
			print('Covariance is high: ',round(self.amcl_cov,5))
			return pt.common.Status.FAILURE
		'''

class Localization(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Setting up Localization!")


		self.cmd_vel_top = "/key_vel"
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
		linear = 0
		angular = 1
		n = 80
		self.move_msg = Twist()
		self.move_msg.linear.x = linear
		self.move_msg.angular.z = angular

		self.service_called = False
		self.i = 0
		self.n = n
        # become a behaviour
		super(Localization, self).__init__("Set up localization!")

	def update(self):

		rate = rospy.Rate(10)
		self.cmd_vel_pub.publish(self.move_msg)
		rate.sleep()
		return pt.common.Status.FAILURE
		'''
		self.i += 1
		if self.i<=self.n:
			print(self.i,'RUNNING')
			return pt.common.Status.RUNNING

		if self.i>self.n:
			self.i = 0
			print('Rotated Enough, Re-Initialising!')
			return pt.common.Status.SUCCESS
		'''
		

class movehead(pt.behaviour.Behaviour):
	"""
	Lowers or raises the head of the robot.
	Returns running whilst awaiting the result,
	success if the action was succesful, and v.v..
	"""

	def __init__(self, direction):

		rospy.loginfo("Initialising move head behaviour.")

        # server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
		self.direction = direction

        # execution checker
		self.tried = False
		self.done = False

		self.num_of_resets = 0

        # become a behaviour
		super(movehead, self).__init__("Lower head!")

	def update(self):

		# Reset checker, if it has to be executed again
		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.tried = False
			self.done = False
			self.num_of_resets = num_of_resets

        # success if done
		if self.done:
			return pt.common.Status.SUCCESS

		# try if not tried
		elif not self.tried:
			self.move_head_req = self.move_head_srv(self.direction)
			self.tried = True

            # tell the tree you're running
			return pt.common.Status.RUNNING

        # if succesful
		elif self.move_head_req.success:
			self.done = True
			return pt.common.Status.SUCCESS

        # if failed
		elif not self.move_head_req.success:
			return pt.common.Status.FAILURE

        # if still trying
		else:
			return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

	"""
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

	def __init__(self):
		rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
		self.goal = PlayMotionGoal()
		self.goal.motion_name = 'home'
		self.goal.skip_planning = True

        # execution checkes
		self.sent_goal = False
		self.finished = False

		self.num_of_resets = 0

        # become a behaviour
		super(tuckarm, self).__init__("Tuck arm!")

	def update(self):

		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.sent_goal = False
			self.finished = False
			self.num_of_resets = num_of_resets

        # already tucked the arm
		if self.finished:
			return pt.common.Status.SUCCESS

        # command to tuck arm if haven't already
		elif not self.sent_goal:

            # send the goal
			self.play_motion_ac.send_goal(self.goal)
			self.sent_goal = True

            # tell the tree you're running
			return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
		elif self.play_motion_ac.get_result():

            # than I'm finished!
			self.finished = True
			return pt.common.Status.SUCCESS

        # if failed
		elif not self.play_motion_ac.get_result():
			return pt.common.Status.FAILURE

        # if I'm still trying :|
		else:
			return pt.common.Status.RUNNING

class counter_in_loop(pt.behaviour.Behaviour):

	"""
    Returns running for n ticks and success thereafter.
    """

	def __init__(self, n, name):

		rospy.loginfo("Initialising counter behaviour.")

        # counter
		self.i = 0
		self.n = n

		self.num_of_resets = 0

        # become a behaviour
		super(counter_in_loop, self).__init__(name)

	def update(self):

		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.i = 0
			self.num_of_resets = num_of_resets

        # increment i
		self.i += 1

        # succeed after count is done
		return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS

class Navigation(pt.behaviour.Behaviour):
	def __init__(self,pick_or_place = 'pick'):
		rospy.loginfo("############## Initialising Navigate Node ################")
		self.move_base = actionlib.SimpleActionClient('/move_base', mb_msgs.MoveBaseAction)
		rospy.loginfo("############## Waiting for server ################")
		self.move_base.wait_for_server()
		rospy.loginfo("############## Connected to SimpleActionServer 'move_base' #################")

		self.pick_or_place = pick_or_place

		self.pick_pose_topic_name = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
		self.place_pose_topic_name = rospy.get_param(rospy.get_name()+ '/place_pose_topic')

		self.pick_pose = None
		self.place_pose = None

		self.pick_pose_topic = rospy.Subscriber(self.pick_pose_topic_name, PoseStamped, self.callback_pick)
		self.place_pose_topic = rospy.Subscriber(self.place_pose_topic_name,PoseStamped,self.callback_place)
		rospy.sleep(5)

		self.flag = False
		if not self.pick_pose:
			raise Exception('####### Did not subscribe to Pick Topic!! #######')

		if not self.place_pose:
			raise Exception('####### Did not subscribe to Place Topic!! #######')


		if pick_or_place == 'place':
			self.goal = mb_msgs.MoveBaseGoal(target_pose = self.place_pose)
		else:
			self.goal = mb_msgs.MoveBaseGoal(target_pose = self.pick_pose)

		#execution checker
		self.sent_goal = False
		self.finished = False

		self.num_of_resets = 0

		# become the behaviour
		super(Navigation, self).__init__("do Navigation stuff!")

	def callback_pick(self,data):
		#print('********* DATA received ************')
		self.flag=True
		self.pick_pose = data
		#print(self.pick_pose)

	def callback_place(self,data):
		self.place_pose = data
		#print(self.place_pose)

	def update(self):
		#print("######## Inside Navigation Update ########")

		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.sent_goal = False
			self.finished = False
			self.num_of_resets = num_of_resets

        # Navigation Already done
		if self.finished:
			#print('Navigation Success initial ifff')
			return pt.common.Status.SUCCESS

        # command to tuck arm if haven't already
		elif not self.sent_goal:

            # send the goal
			self.move_base.send_goal(self.goal)
			print('############# Sent the Goal ##############')
			self.sent_goal = True

            # tell the tree you're running
			print('Navigation Running')
			return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
		elif self.move_base.get_result():

            # than I'm finished!
			self.finished = True
			print('Navigation Success')
			return pt.common.Status.SUCCESS

        # if failed
		elif not self.move_base.get_result():
			# print('Navigation Failure')
			return pt.common.Status.FAILURE

        # if I'm still trying :|
		else:
			print('Navigation Running')
			return pt.common.Status.RUNNING

class PickCube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Initialising picking.")
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		rospy.wait_for_service(self.pick_srv_nm, timeout = 20)

		self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

		self.finished = False
		self.tried = False

		self.num_of_resets = 0

        # become a behaviour
		super(PickCube, self).__init__("Pick cube!")

	def update(self):

		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.finished = False
			self.tried = False
			self.num_of_resets = num_of_resets

        # finished
		if self.finished:
			return pt.common.Status.SUCCESS

        # try
		elif not self.tried:
			self.reply = self.pick_srv()
			self.tried = True

            # tell the tree you're running
			return pt.common.Status.RUNNING

        # if I was succesful!
		elif self.reply.success:

            # than I'm finished!
			self.finished = True
			print("finished picking")
			rospy.sleep(2)
			return pt.common.Status.SUCCESS

        # if failed
		elif not self.reply.success:
			return pt.common.Status.FAILURE

        # if I'm still trying :|
		else:
			return pt.common.Status.RUNNING


class PlaceCube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Initialising placing.")
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		rospy.wait_for_service(self.place_srv_nm, timeout = 20)

		self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

		self.finished = False
		self.tried = False

		self.num_of_resets = 0

        # become a behaviour
		super(PlaceCube, self).__init__("Place cube!")

	def update(self):

		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.finished = False
			self.tried = False
			self.num_of_resets = num_of_resets

        # finished
		if self.finished:
			return pt.common.Status.SUCCESS

        # try
		elif not self.tried:
			self.reply = self.place_srv()
			self.tried = True

            # tell the tree you're running
			return pt.common.Status.RUNNING

        # if I was succesful!
		elif self.reply.success:

            # than I'm finished!
			self.finished = True
			return pt.common.Status.SUCCESS

        # if failed
		elif not self.reply.success:
			return pt.common.Status.FAILURE

        # if I'm still trying :|
		else:
			return pt.common.Status.RUNNING

class LocalizationSetup(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Setting up localization!")

		self.randomise_particles_srv_name = rospy.get_param(rospy.get_name() + '/global_loc_srv') #from the amcl documentation

		# Connect to Services
		rospy.wait_for_service(self.randomise_particles_srv_name, timeout = 20)
		self.randomise_particles_srv = rospy.ServiceProxy(self.randomise_particles_srv_name, Empty)

		self.done = False

        # become a behaviour
		super(LocalizationSetup, self).__init__("Set up localization!")

	def update(self):

        # finished
		if self.done:
			return pt.common.Status.SUCCESS

        # try
		if not self.done:
			self.reply = self.randomise_particles_srv()
			self.done = True

			return pt.common.Status.SUCCESS



class Check(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Setting up Check Node!")
		self.cube_pose = PoseStamped()
		rospy.sleep(3)

		self.cube_detected = False
		self.cube_pose_nm = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

		# Counts how many times cube was not detected.
		# This is needed because it might take a while in the beginning until the robot
		# recognizes the cube
		self.counter = 0

		#Respawn Service
		self.respawn_srv_nm = '/gazebo/set_model_state'

		rospy.wait_for_service(self.respawn_srv_nm)
		self.respawn_srv = rospy.ServiceProxy(self.respawn_srv_nm, SetModelState)
		rospy.loginfo("Connected to Respawn Service!")

		state_msg = ModelState()
		state_msg.model_name = 'aruco_cube'
		state_msg.pose.position.x = -1.130530
		state_msg.pose.position.y = -6.653650
		state_msg.pose.position.z =  0.86250
		state_msg.pose.orientation.x = 0
		state_msg.pose.orientation.y = 0
		state_msg.pose.orientation.z = 0
		state_msg.pose.orientation.w = 1

		state_msg.twist.linear.x = 0
		state_msg.twist.linear.y = 0
		state_msg.twist.linear.z = 0
		state_msg.twist.angular.x = 0
		state_msg.twist.angular.y = 0
		state_msg.twist.angular.z = 0
		state_msg.reference_frame = 'map'

		self.cube_pose_message = state_msg

		self.num_of_resets = 0


        # become a behaviour
		super(Check, self).__init__("Check!")

	def callback(self, data):
		#print("Callback is called")

		if data.pose.position.x != None:
			self.cube_detected = True

	def update(self):
		global num_of_resets
		if num_of_resets > self.num_of_resets:
			self.cube_detected = False
			self.num_of_resets = num_of_resets
			self.counter = 0

		self.aruco_pose = rospy.Subscriber(self.cube_pose_nm, PoseStamped, self.callback)

        # cube is on table
		if self.cube_detected:
			print(' ********** Cube Detected, Task Accomplished! Hope we get that A now! :) ************')
			self.counter = 0
			return pt.common.Status.SUCCESS
		#cube is not on table
		elif not self.cube_detected:
			print(' waiting for cube detection')
			self.counter += 1

			if self.counter > 50:
				print(' ********** RESTART TASK ********** ')
				global num_of_resets
				num_of_resets += 1
				self.respawn_srv(self.cube_pose_message)

			return pt.common.Status.FAILURE

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()