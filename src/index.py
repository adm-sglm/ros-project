import sys
from flask import request, jsonify
import redis
import json
from flask import Flask
from flask import render_template
import threading
from flask_socketio import SocketIO
from flask_socketio import send,emit
#from movebase import MoveBase

r = redis.Redis()

p = r.pubsub()
# p.subscribe('ros-web')

# initialdata_got = False
# while initialdata_got == False:
# 	print('no initialdata')
# 	message = p.get_message()
# 	if (message):
# 		print(message)
# 		initialdata_got = True

posdata = None

def mhandler(data):
	global posdata
	posdata = data
	pos = json.loads(posdata['data'].decode('utf-8'))
	socketio.emit('pos-refresh',pos)

def goalhandler(data):
	message = data['data'].decode('utf-8')
	print(message)
	socketio.emit('goal_status',message)

p.subscribe(**{'ros-amcl': mhandler})
p.subscribe(**{'ros-goalstatus': goalhandler})
thread = p.run_in_thread(sleep_time=0.5)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'rosros'
socketio = SocketIO(app)

@app.route("/")
def hello():
	return render_template('main.html')

@app.route("/cmd/start")
def setup():
	r.publish('ros-panel',json.dumps({"cmd": "move","point": "test"}))
	return "start"

@app.route("/cmd/setinit")
def setinit():
	x = request.args.get('x')
	y = request.args.get('y')
	angle = request.args.get('ang')
	# print(x,file=sys.stdout)
	r.publish('ros-panel',json.dumps({"cmd": "set_initial_pose","pose": {"x": x,"y": y,"angle": angle}}))
	return x	

@app.route("/cmd/run",methods=["POST"])
def run():
	data = request.get_json()
	r.publish('ros-panel',json.dumps({"cmd": "move","location": data["location"]}))
	return "run"

@app.route('/cmd/map',methods=["POST"])
def get_map():
	with open('map_metadata.json') as json_file:
		data = json.load(json_file)
		return json.dumps(data)

@app.route('/cmd/pos',methods=["GET"])
def get_pos():
	global posdata
	pos = json.loads(posdata['data'].decode('utf-8'))
	socketio.emit('pos-refresh',pos)
	return json.dumps(pos)

@socketio.on('connect')
def connect():
	print('connect')
	
if __name__ == '__main__':
	socketio.run(app)