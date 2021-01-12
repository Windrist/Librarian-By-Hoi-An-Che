#!/usr/bin/env python3

import speech_recognition as sr
import rospy
import time
from std_msgs.msg import String
from lib_chatbot.msg import Chatbot

class SoundRecognition(object):
    
    def __init__(self):
        
        # Init ROS
        rospy.init_node('mic_node', anonymous = True)
        self.pub_mic = rospy.Publisher("/Mic_state", Chatbot, queue_size=100)
        self.pub_state = rospy.Publisher("/Main_state", String, queue_size=100)
        self.pub_play = rospy.Publisher("/Play_file", String, queue_size=100)
        rospy.Subscriber("/Main_state", String, self.callback)
        self.rate = rospy.Rate(10)

        # Init Variable
        self.file = os.path.dirname(__file__)
        self.main = String()
        self.mic = Chatbot()
        self.play = String()
        self.state = "Idle"
        self.json_key = open(self.file + '/../file/Librarian.json').read()
        self.r = sr.Recognizer()
        self.r.energy_threshold = 300
        self.count_again = 0
        self.check_noise = True

    def get_cmd(self):
        # Get Keyword Command to start ChatBot
        try:
            mic = sr.Microphone(sample_rate = 48000)
            with mic as s:

                # Check Noise Before Listen
                if self.check_noise == True:
                    self.r.adjust_for_ambient_noise(s, duration=3)
                    print("Check Noise Done!")
                    self.check_noise = False

                # Listen
                audio = self.r.listen(s)
                print("Got Something!")

                # Get Text from Google API
                text = self.r.recognize_google(audio, None, "vi-VN")
                text = text.lower()
                print("I heard: " + text)

                # Send Text to Main Node
                self.mic.data = text
                self.mic.state = "Got"
                self.pub_mic.publish(self.mic)
        except sr.UnknownValueError:
            # Skip with no value Speech
            print("Oops! Didn't catch that")
            pass
        except sr.RequestError as e:
            # Send Error if Disconnect
            print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
            self.main.data = "Error"
            self.pub_state.publish(self.main)
            pass

    def sound_processing(self):
        # Speech to Text Processing
        try:
            mic = sr.Microphone(sample_rate = 48000)
            with mic as s:

                # Listen
                audio = self.r.listen(s)
                print("Got Something!")

                # Get Text from Google API
                text = self.r.recognize_google(audio, None, "vi-VN")
                text = text.lower()
                print("I heard: " + text)

                # Send Text to Main Node
                self.mic.data = text
                self.mic.state = "Got"
                self.pub_mic.publish(self.mic)
        except sr.UnknownValueError:
            if self.count_again == 3:
                # End if no Response Third Time
                self.count_again = 0
                self.main.data = "End_With_Silent"
                self.pub_state.publish(self.main)
            else:
                # Skip with no value Speech
                print("Oops! Didn't catch that! Nói lại đi!")
                self.count_again += 1
                self.main.data = "Again"
                self.pub_state.publish(self.main)
            pass
        except sr.RequestError as e:
            # Send Error if Disconnect
            print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
            self.main.data = "Error"
            self.pub_state.publish(self.main)
            pass

    def callback(self, msg):
        # Get State to Listen from Main Node
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