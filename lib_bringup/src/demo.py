#!/usr/bin/env python3

import rospy
import os
import time
from std_msgs.msg import String
from chatbot.msg import Chatbot
import sys

class Demo(object):
    
    def __init__(self):
        rospy.init_node('demo', anonymous = True)
        self.pub_chat = rospy.Publisher("/Chat_state", Chatbot, queue_size=100)
        rospy.Subscriber("/Mic_state", Chatbot, self.callbackMic)
        self.rate = rospy.Rate(10)

        self.chat = Chatbot()

    def callbackMic(self, msg):
        if msg.state == "Got" and msg.data == u"xin chào":
            self.chat.data = "chào bạn"
            self.chat.state = "Greeting"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"mình muốn mượn sách":
            self.chat.data = ""
            self.chat.state = "Borrow Book"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"mình muốn mượn sách đại số":
            self.chat.data = "đại số"
            self.chat.state = "Book with Name"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"đại số":
            self.chat.data = "đại số"
            self.chat.state = "Book with Name"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"điện quang":
            self.chat.data = "điện quang"
            self.chat.state = "Book with Name"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"thực tập điện tử tương tự":
            self.chat.data = "thực tập điện tử tương tự"
            self.chat.state = "Book with Name"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"trần hữu quốc đông":
            self.chat.data = "trần hữu quốc đông"
            self.chat.state = "Name"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"có":
            self.chat.data = ""
            self.chat.state = "Confirm"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"không":
            self.chat.data = ""
            self.chat.state = "Deny"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"mình muốn trả sách":
            self.chat.data = ""
            self.chat.state = "Return Book"
            self.pub_chat.publish(self.chat)
        if msg.state == "Got" and msg.data == u"đông có đẹp trai hay không":
            self.chat.data = "tất nhiên là có rồi"
            self.chat.state = "Chitchat"
            self.pub_chat.publish(self.chat)
        


    def spin(self):     
        while not rospy.is_shutdown():
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    Demo = Demo()
    Demo.spin()