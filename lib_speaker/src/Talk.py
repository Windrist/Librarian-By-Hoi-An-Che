#!/usr/bin/env python3

import os
import rospy
import time
from std_msgs.msg import String
import requests
import urllib
import sys
import subprocess
from google.cloud import texttospeech
import subprocess

class Speak(object):
	def __init__(self):
		self.file = os.path.dirname(__file__)
		rospy.init_node('Talker', anonymous=True)
		self.pub_state = rospy.Publisher("/Main_state", String, queue_size=100)
		rospy.Subscriber("/Talk_text", String, self.callback)
		self.rate = rospy.Rate(10)

		self.api_url = 'https://api.fpt.ai/hmi/tts/v5'
		self.headers = {
			'api-key': '5TAfPcI66laURvEauXn8VZYbMGLl845M',
			'speed': '-1',
			'voice': 'banmai'
		}
		self.client = texttospeech.TextToSpeechClient()

		self.text = ""
		self.payload = String()

		rospy.spin()

	def fptai(self):
		# response = requests.request('POST', self.api_url, data=self.text.encode('utf-8'), headers=self.headers)
		# content = dict(response.json())
		# url = content['async']
		# url = url.replace('https','http')

		# while True:
		# 	try:
		# 		urllib.request.urlopen(url)
		# 	except urllib.error.HTTPError:
		# 		pass
		# 	except urllib.error.URLError:
		# 		pass
		# 	else:
		# 		break

		synthesis_input = texttospeech.SynthesisInput(text=self.text.encode('utf-8'))
		voice = texttospeech.VoiceSelectionParams(language_code="vi-VN", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL)
		audio_config = texttospeech.AudioConfig(audio_encoding=texttospeech.AudioEncoding.MP3)
		response = self.client.synthesize_speech(input=synthesis_input, voice=voice, audio_config=audio_config)
		with open(self.file + "/../file/Talk.mp3", "wb") as out:
			# Write the response to the output file.
			out.write(response.audio_content)
		print("Speaking: " + self.text)

		cmd = "mpg123 -q " + self.file + "/../file/Talk.mp3"
		subprocess.run(cmd, shell=True)

		# cmd = "mpg123 -q '{}'".format(url)
		# subprocess.run(cmd, shell=True)
		self.payload.data = "Done"
		self.pub_state.publish(self.payload)

	def callback(self, msg):
		self.text = msg.data
		print("Ok! I need to speak!")
		self.fptai()

	def spin(self):
		while not rospy.is_shutdown():
			self.rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	Speak = Speak()
	Speak.spin()