# -*- coding: utf-8 -*-
"""
Created on Wed Aug  5 19:18:09 2020

@author: Aya
"""

import rospy
from std_msgs.msg import String


from flask import Flask, render_template,jsonify
from flask_socketio import SocketIO,emit


rospy.init_node('app', anonymous=True)
pub = rospy.Publisher('app_to_benzo', String, queue_size = 3)
msg_string = String()

app = Flask(__name__)
socketio = SocketIO(app,ping_timeout=30,ping_interval=30)

def listener():
    rospy.Subscriber('benzo_to_app', String, broadcast)

@socketio.on('req')
def handle_message(message):
    print('received message: ' + message)
    pub.publish(message)
    #broadcast(message)

@socketio.on('connect')
def test_connect():
    print('connection received')
    emit('myres', {'data': 'Connected'})

@socketio.on('broadcast')
def broadcast(message):
    print('broadcast initiated')
    emit('rec',{'data':message},broadcast=True)
 

if __name__ == '__main__':
    socketio.run(app,debug=False, host='0.0.0.0',port=8000)
