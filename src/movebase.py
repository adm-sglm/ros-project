import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
import numpy as np
from math import radians, pi
import json
from nav_msgs.srv import GetMap
import png



# Point y (2nd param) horizontal place in map --

setup_callback = None
current_pos = None
pos_handler = None
pos_pub = None

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

class MoveBase():
	def __init__(self):
		rospy.init_node('nav_test', anonymous=False)
		rospy.on_shutdown(self.shutdown)
		
		self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
		#self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)

		rospy.loginfo("Waiting action server...")
		self.move_base.wait_for_server(rospy.Duration(60))
		rospy.loginfo("Connected to server")

		global pos_pub
		pos_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

		rospy.sleep(2)

		self.locations = dict()		

		self.locations['entrance'] = [Point(-1.486,7.676,-0.624),pi/180*330]
		self.locations['tv'] = [Point(2.777,9.079,-1.633),3*pi/2]
		self.locations['watertank'] = [Point(4.713,2.567,2.525),pi/180*130]
		self.locations['startingpos'] = [Point(1.879, 3.291, 1.909),pi/180*270]

	def run2(self):
		square_size = rospy.get_param("~square_size", 1.0)
		goals = [[Point(square_size,0.0,0.0),pi/2],[Point(square_size,square_size,0.0),pi],[Point(0.0,square_size,0.0),3*pi/2],[Point(0.0,0.0,0.0),0]]

		orders = list()
		for goal in goals:
			mb_goal = MoveBaseGoal()	
			mb_goal.target_pose.header.frame_id = 'map'			
			orientation = Quaternion(euler_to_quaternion(0,0,goal[1]))
			pose = Pose(goal[0],orientation)
			mb_goal.target_pose.pose = pose
			orders.append(mb_goal)
		
		for order in orders:
			self.move(order)

	def run(self,location_name):
		goal_location = self.locations[location_name]
		goal = self.CreateGoalInst(goal_location[0],goal_location[1])

		self.move(goal)

	def run_to_point(self,x,y,z,angle,cb):		
		goal_point = Point(float(x), float(y), float(z))
		goal_angle = eval(angle)
		
		goal = self.CreateGoalInst(goal_point,goal_angle)		
		self.move(goal,cb)

	def set_initialpose(self,x,y,z,angle):			
		initialpose = PoseWithCovarianceStamped()		
		initialpose.header.stamp = rospy.Time.now()
		initialpose.header.frame_id = 'map'		
		
		initialpose.pose.pose.position = Point(float(x),float(y),z)
		
		rad_angle = int(angle)*pi/180
		
		initialpose.pose.pose.orientation = Quaternion(*euler_to_quaternion(0,0,rad_angle))
		print(initialpose)
		pos_pub.publish(initialpose)

	def setup(self,cb):
		global setup_callback
		setup_callback = cb
		
		# rospy.wait_for_service('static_map')
		# mapsrv = rospy.ServiceProxy('static_map', GetMap)
		# resp = mapsrv()		

		# rospy.Subscriber('map_metadata',MapMetaData,self.GetMapMetaData)
		rospy.Subscriber('map',OccupancyGrid,self.GetMapData)
		rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,self.SubPosition)


		# print("setup ran")
		# #rospy.Subscriber('initialpose', PoseWithCovarianceStamped,self.update_initial_pose)
		# self.ipose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

		# rospy.sleep(2)
		# print(initialdata)
			
		# initialpose = PoseWithCovarianceStamped()		
		# initialpose.header.frame_id = 'map'		
		
		# # # initialpose.pose.pose.position.x = 2.5
		# # # initialpose.pose.pose.position.y = 3.15
		# # # initialpose.pose.pose.position.z = 0.0

		# initialpose.pose.pose.position = Point(0.0,0.0,0.0)
		# initialpose.pose.pose.orientation = Quaternion(*euler_to_quaternion(0,0,90*pi/180))
		
		# print("publishing")
		# t = pos_pub.publish(initialpose)
		# print(t)
		
		# rospy.sleep(2)

	def GetMapMetaData(self,metadata,mapimgname):
		filename = "map_metadata.json"

		pos = {
			"x": metadata.origin.position.x,
			"y": metadata.origin.position.y,
			"z": metadata.origin.position.z
		}

		orientation = {
			"x": metadata.origin.orientation.x,
			"y": metadata.origin.orientation.y,
			"z": metadata.origin.orientation.z,
			"w": metadata.origin.orientation.w
		}

		csmeta = {
			"width": metadata.width,
			"height": metadata.height,
			"resolution": metadata.resolution,
			"pos": pos,
			"orientation": orientation,
			"mapimgfile": mapimgname
		}

		f = open(filename, 'w')
		json.dump(csmeta, f)

	def GetMapData(self,mdata):
		print("get map data")
		filename = './static/map.png'
		self.GetMapMetaData(mdata.info,filename)
		width = mdata.info.width
		height = mdata.info.height
		
		occupancy = np.array(mdata.data,dtype=np.int8)

		f = open(filename, 'wb')
		w = png.Writer(width,height,greyscale=True)
		occupancy = np.reshape(occupancy,(height,width))

		w.write(f,occupancy)
		f.close()
		print("map saved map.png")

		# setup_callback(mdata.info,filename)
		#np.save('map',occupancy)
	def SubPosition(self,posedata):
		global current_pos
		global pos_handler

		if (pos_handler):
			pos_handler(posedata)
		# current_pos = {"x": posedata.pose.pose.position.x,"y": posedata.pose.pose.position.y}	

	def attach_pos(self,cb):
		global pos_handler
		pos_handler = cb

	def CreateGoalInst(self,point,orientation):
		mb_goal = MoveBaseGoal()
		mb_goal.target_pose.header.frame_id = 'map'
		ori = Quaternion(*euler_to_quaternion(0,0,orientation))
		pose = Pose(point,ori)
		mb_goal.target_pose.pose = pose
		
		return mb_goal

	def move(self,goal,cb):
		self.move_base.send_goal(goal)

		finished = self.move_base.wait_for_result(rospy.Duration(60))

		if finished:
			state = self.move_base.get_state()
			cb(state)
			if state == GoalStatus.SUCCEEDED:
				# cb(1) you can implement this				
				print("Goal reached")
			
	def update_initial_pose(self,initial_pose):
		print("update initial pose")
		rospy.loginfo(initial_pose)
		self.initial_pose = initial_pose
	def shutdown(self):
		self.move_base.cancel_goal()
		print("shutdown")
		return True