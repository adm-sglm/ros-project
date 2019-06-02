import redis
import json
import threading
import rospy
from movebase import MoveBase as AdmMove

r = redis.Redis()
p = r.pubsub()

p.subscribe('ros-panel')

map_metadata = None
map_filename = None
current_pos = None

def setupdone(minfo,mapfilename):   
    global map_metadata, map_filename
    map_metadata = minfo
    map_filename = mapfilename
    # r.publish('ros-web','setupdone-message')

mb = AdmMove()
mb.setup(setupdone)

def GetPosition(data):  
    current_pos = {
    "pos": {
        "x": data.pose.pose.position.x,
        "y": data.pose.pose.position.y
        },
    "orientation": {
        "x": data.pose.pose.orientation.x,
        "y": data.pose.pose.orientation.y,
        "z": data.pose.pose.orientation.z,
        "w": data.pose.pose.orientation.w
        }
    }
    r.publish('ros-amcl',json.dumps(current_pos))

mb.attach_pos(GetPosition)

def move_finished(status):  
    r.publish('ros-goalstatus',status)    


def process_message(message):
  if message["type"] == 'message':
    data = json.loads(message["data"].decode('utf-8'))
    cmd = data["cmd"]

    if cmd == "move":
        location = data["location"]
        mb.run_to_point(location["x"],location["y"],location["z"],location["angle"],move_finished)
    if cmd == "set_initial_pose":
        mb.set_initialpose(data["pose"]["x"],data["pose"]["y"],0.0,data["pose"]["angle"])
    
    # if cmd == "request":
    #     r.publish('ros-web','request done')
    
    if message["type"] == "mapdata":
        global map_metadata, map_filename
        data = json.loads({metadata: map_metadata})
        r.publish('ros-panel-startup','worker-test')
  # if message["type"] == "position":
  #     global current_pos
  #     r.publish('ros-web',json.loads())

try:
    while True:
        message = p.get_message()
        if (message):            
            process_message(message)
except KeyboardInterrupt:
    print("quitting")