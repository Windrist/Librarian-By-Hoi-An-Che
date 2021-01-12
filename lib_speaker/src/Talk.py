#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
import subprocess
from google.cloud import texttospeech

class Speak(object):
	def __init__(self):

		# Init ROS
		rospy.init_node('Talker', anonymous=True)
		self.pub_state = rospy.Publisher("/Main_state", String, queue_size=100)
		rospy.Subscriber("/Talk_text", String, self.callback)
		self.rate = rospy.Rate(10)

		# Declare Variable
		self.file = os.path.dirname(__file__)
		self.client = texttospeech.TextToSpeechClient()
		self.text = ""
		self.payload = String()

	def processing(self):
		
		# Text To Speech Init
		synthesis_input = texttospeech.SynthesisInput(text=self.text.encode('utf-8'))
		voice = texttospeech.VoiceSelectionParams(language_code="vi-VN", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL)
		audio_config = texttospeech.AudioConfig(audio_encoding=texttospeech.AudioEncoding.MP3)

		# Get Response
		response = self.client.synthesize_speech(input=synthesis_input, voice=voice, audio_config=audio_config)
		
		# Write the response to the output file
		with open(self.file + "/../file/Talk.mp3", "wb") as out:
			out.write(response.audio_content)

		# Play File
		print("Speaking: " + self.text)
		cmd = "mpg123 -q " + self.file + "/../file/Talk.mp3"
		subprocess.run(cmd, shell=True)

		# Send Message to Main Node
		self.payload.data = "Done"
		self.pub_state.publish(self.payload)

	def callback(self, msg):
		# Get Text from Message
		self.text = msg.data
		print("Ok! I need to speak!")
		self.processing()

	def spin(self):
		while not rospy.is_shutdown():
			self.rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	Speak = Speak()
	Speak.spin()