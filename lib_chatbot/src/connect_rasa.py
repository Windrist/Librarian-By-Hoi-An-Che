#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from lib_chatbot.msg import Chatbot
import requests

class ConnectRasa(object):
    def __init__(self):

        # Init ROS
        rospy.init_node('connect_rasa', anonymous=True)
        self.pub = rospy.Publisher('/Chat_state', Chatbot, queue_size=10)
        rospy.Subscriber("/Voice_recog", String, self.callback)
        self.rate = rospy.Rate(10)

        # Init Variable
        self.payload = Chatbot()
        self.msg = ""
        self.voice_text = ''

    def get_text(self):
        # Get Response Text and State from Rasa
        voice_data = u"{}".format(self.voice_text)
        req = requests.post("http://localhost:5002/webhooks/rest/webhook", json = {'message': voice_data})
        for i in req.json():
            self.msg = i['text']
        state = self.msg.split("-")
        
        # Split Data, Send Text and State
        self.payload.state = state[0]
        self.payload.data = u"{}".format(state[1])
        self.pub.publish(self.payload)

    def callback(self, msg):
        # Get Input Text
        self.voice_text = msg.data
    
    def spin(self):
        while not rospy.is_shutdown(): 
            if self.voice_text != "":
                self.get_text()
                self.voice_text = ""
        rospy.spin()

if __name__ == '__main__':
    ConnectRasa = ConnectRasa()
    ConnectRasa.spin()

        
        
           
    