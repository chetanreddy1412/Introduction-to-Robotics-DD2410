#!/usr/bin/env python

# group members: Chetan Reddy Narayanaswamy, Florentina Voboril

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):
		# Access rosparams
		self.marker_pose_topic_name = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

		# Instantiate publishers
		self.cube_pose_pub = rospy.Publisher(self.marker_pose_topic_name, PoseStamped, queue_size=1000) 

		rospy.loginfo("Initialising behaviour tree")

		# lower head
		b_movehead = movehead("down")

		# tuck the arm
		b_tuck = tuckarm()

		print("**************before pick cube")
		# pick
		b_pick = PickCube()

		print("**************after pick cube")

		# turn
		b_move1 = pt.composites.Selector(
			name="Turn fallback",
			children=[counter(33, "Turned?"), go("Turn!", 0, -0.9)]
		)
		# Pausing 
		b_move2 = pt.composites.Selector(
			name="Pause",
			children=[counter(10, "At table?"), go("Stay Still for a Second", 0, 0)]
		)

		# go to table 2
		b_move3 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(12, "At table?"), go("Go to table!", 0.7, 0)]
		)

		b_move = pt.composites.Sequence(
			name="Movement to Table 2",
			children=[b_move1,b_move2,b_move3]
		)

		# place
		b_place = PlaceCube()

		b_check = Check()
		
		b_place_check = pt.composites.Sequence(
			name="Place Check Sequence",
			children = [b_place,b_check]
		)
		# turn back
		b_goback1 = pt.composites.Selector(
			name="Turn back fallback",
			children=[counter(33, "Turned back?"), go("Turn back!", 0, 1)]
		)

		#Pause
		b_goback2 = pt.composites.Selector(
			name="Pause",
			children=[counter(10, "At table?"), go("Pause for a second", 0, 0)]
		)
		# go back to table 1
		b_goback3 = pt.composites.Selector(
			name="Go back to table fallback",
			children=[counter(12, "At table?"), go("Go back to table!", 0.7, 0)]
		)

		b_goback = pt.composites.Sequence(
			name="Movement to Table 1",
			children=[b_goback1,b_goback2, b_goback3]
		)
		b_lastnode = pt.composites.Selector(
			name='Checking if Successful..',
			children=[b_place_check,b_goback]
		)



		# become the tree
		tree = RSequence(name="Main sequence", children = [b_movehead,b_tuck,b_pick,b_move,b_lastnode])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

class PickCube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Initialising picking.")
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		rospy.wait_for_service(self.pick_srv_nm, timeout = 20)

		print("**************before pick service proxy******************")
		self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
		print("**************after pick service proxy******************")

		self.finished = False
		self.tried = False

        # become a behaviour
		super(PickCube, self).__init__("Pick cube!")
		
	def update(self):

        # finished
		if self.finished: 
			return pt.common.Status.SUCCESS
        
        # try
		elif not self.tried:
			print('********************** before calling pick function handle *****************')
			self.reply = self.pick_srv()
			print('********************** after pick function handle ****************')
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


class PlaceCube(pt.behaviour.Behaviour):
	def __init__(self):
		rospy.loginfo("Initialising placing.")
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		rospy.wait_for_service(self.place_srv_nm, timeout = 20)

		print("**************before pick service proxy***************")
		self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
		print("**************after place service proxy*****************")

		self.finished = False
		self.tried = False

        # become a behaviour
		super(PlaceCube, self).__init__("Place cube!")
		
	def update(self):

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

class Check(pt.behaviour.Behaviour):
	def __init__(self):
		self.cube_pose = PoseStamped()
		rospy.sleep(3)

		self.cube_detected = False
		self.cube_pose_nm = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		

        # become a behaviour
		super(Check, self).__init__("Check!")

	def callback(self, data):
	
		if data.pose.position.x != None:
			self.cube_detected = True
		
	def update(self):
		self.aruco_pose = rospy.Subscriber(self.cube_pose_nm, PoseStamped, self.callback)

        # cube is on table
		if self.cube_detected: 
			print(' ********** Cube Detected!!!!!!!!!!!!!!! ************')
			return pt.common.Status.SUCCESS
		#cube is not on table
		elif not self.cube_detected:
			print(' ********** Cube NOT Detected ************')
			return pt.common.Status.FAILURE

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()