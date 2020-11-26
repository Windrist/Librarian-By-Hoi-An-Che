#!/usr/bin/env python3

import speech_recognition as sr
import rospy
import os
import time
from std_msgs.msg import String
from lib_chatbot.msg import Chatbot
import sys
import logmmse
from record import record_3s, record_5s, record_7s
import requests

class SoundRecognition(object):
    
    def __init__(self):
        rospy.init_node('mic_node', anonymous = True)
        self.pub_mic = rospy.Publisher("/Mic_state", Chatbot, queue_size=100)
        self.pub_state = rospy.Publisher("/Main_state", String, queue_size=100)
        self.pub_play = rospy.Publisher("/Play_file", String, queue_size=100)
        rospy.Subscriber("/Main_state", String, self.callback)
        self.rate = rospy.Rate(10)

        self.file = os.path.dirname(__file__)
        self.main = String()
        self.mic = Chatbot()
        self.play = String()
        self.state = "Idle"

        self.json_key = open(self.file + '/../file/Librarian.json').read()
        self.r = sr.Recognizer()
        # self.r.energy_threshold = 300
        self.count_again = 0
        self.check_noise = True

    def get_cmd(self):
        try:
            mic = sr.Microphone(sample_rate = 48000)
            with mic as s:
                if self.check_noise == True:
                    self.r.adjust_for_ambient_noise(s, duration=3)
                    print("Check Noise Done!")
                    self.check_noise = False
                audio = self.r.listen(s)
                print("Got Something!")
                # text = self.r.recognize_google_cloud(audio, self.json_key, "vi-VN")
                text = self.r.recognize_google(audio, None, "vi-VN")
                text = text.lower()
                print("I heard: " + text)
                self.mic.data = text
                self.mic.state = "Got"
                self.pub_mic.publish(self.mic)
        except sr.UnknownValueError:
            print("Oops! Didn't catch that")
            pass
        except sr.RequestError as e:
            print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
            self.main.data = "Error"
            self.pub_state.publish(self.main)
            pass

    def sound_processing(self):
        # if self.state == "Record5":
        #     record_5s()
        # elif self.state == "Record3":
        #     record_3s()
        # elif self.state == "Record7":
        #     record_7s()
        try:
            # with sr.AudioFile(self.file + "/../file/Get.wav") as file:
            mic = sr.Microphone(sample_rate = 48000)
            with mic as s:
                # audio = self.r.record(file)
                audio = self.r.listen(s)
                print("Got Something!")
                # text = self.r.recognize_google_cloud(audio, self.json_key, "vi-VN")
                text = self.r.recognize_google(audio, None, "vi-VN")
                text = text.lower()
                print("I heard: " + text)
                self.mic.data = text
                self.mic.state = "Got"
                self.pub_mic.publish(self.mic)
        except sr.UnknownValueError:
            if self.count_again == 3:
                self.count_again = 0
                self.main.data = "End_With_Silent"
                self.pub_state.publish(self.main)
            else:
                print("Oops! Didn't catch that! Nói lại đi!")
                self.count_again += 1
                self.main.data = "Again"
                self.pub_state.publish(self.main)
            pass
        except sr.RequestError as e:
            print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
            self.main.data = "Error"
            self.pub_state.publish(self.main)
            pass

    def callback(self, msg):
        self.state = msg.data
        if self.state == "Idle":
            self.check_noise = True
        if self.state == "Record3" or self.state == "Record5" or self.state == "Record7":
            self.sound_processing()

    def spin(self):     
        while not rospy.is_shutdown():
            if self.state == "Idle":
                self.get_cmd()
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    SoundRecognition = SoundRecognition()
    SoundRecognition.spin()