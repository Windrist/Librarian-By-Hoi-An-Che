#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from lib_chatbot.msg import Chatbot
from database import checkBook, checkName, borrowBook, returnBook
import sys

class StateMachine(object):
    
    def __init__(self):
        rospy.init_node('main_node', anonymous = True)
        self.pub_main = rospy.Publisher("/Main_state", String, queue_size=100)
        self.pub_mic = rospy.Publisher("/Mic_state", Chatbot, queue_size=100)
        self.pub_recog = rospy.Publisher("/Voice_recog", String, queue_size=100)
        self.pub_chat = rospy.Publisher("/Chat_state", Chatbot, queue_size=100)
        self.pub_talk = rospy.Publisher("/Talk_text", String, queue_size=100)
        self.pub_play = rospy.Publisher("/Play_file", String, queue_size=100)
        self.pub_nav = rospy.Publisher("/Nav_point", Pose2D, queue_size=100)
        rospy.Subscriber("/Main_state", String, self.callbackState)
        rospy.Subscriber("/Mic_state", Chatbot, self.callbackMic)
        rospy.Subscriber("/Chat_state", Chatbot, self.callbackChat)
        self.rate = rospy.Rate(10)

        self.main = String()
        self.recog = String()
        self.talk = String()
        self.play = String()
        self.nav = Pose2D()
        self.state = "Idle"
        self.prev_state = ""
        self.borrow_state = False
        self.return_state = False
        self.chitchat_state = False

        self.book_name = ""
        self.author = ""
        self.amount = ""
        self.column = ""
        self.row = ""

        self.return_book_name = []
        self.name = ""

    def callbackState(self, msg):
        if msg.data == "Done" and self.state == "@Greeting":
            self.state = "Ask_Start"
            self.play.data = "Ask_Start.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "Ask_Start":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "Help":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "@Borrow_Book":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "@Return_Book":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "Ask_Name":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "@Check_Borrow":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "@Check_Return":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "Manipulator":
            self.state = "Record3"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "Wait_Me":
            self.state = "Navigation"
            self.main.data = self.state
            self.nav.x = self.column
            self.nav.y = self.row
            self.pub_main.publish(self.main)
            self.pub_nav.publish(self.nav)
        elif msg.data == "Done" and (self.state == "Navigation" or self.state == "Good_Luck"):
            borrowBook(self.name, self.book_name, self.author)
            self.state = "End_Borrow_Book"
            self.play.data = "End_Borrow_Book.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "@Chit_Chat":
            self.state = "Ask_Again"
            self.play.data = "Ask_Again.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "@aboutrobot":
            self.state = "Ask_Again"
            self.play.data = "Ask_Again.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "Ask_Again":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "Affirm_Chitchat":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "End_With_Silent":
            self.state = "End_With_Silent"
            self.play.data = "End_With_Silent.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Again":
            self.prev_state = self.state
            self.state = "Again"
            self.play.data = "Again_Because_Silent.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "Again":
            self.state = self.prev_state
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Done" and self.state == "@rephrase":
            self.state = "Record5"
            self.main.data = self.state
            self.pub_main.publish(self.main)
        elif msg.data == "Error":
            self.state = "Error"
            self.play.data = "Error.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "Out_Of_Book":
            self.state = "End_Out_Book"
            self.play.data = "End_Common.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and self.state == "Correct_Borrow_Book":
            self.state = "Manipulator"
            self.play.data = "Manipulator.mp3"
            self.pub_play.publish(self.play)
        elif msg.data == "Done" and (self.state == "End_Borrow_Book" or self.state == "End_Return_Book" or self.state == "End_Out_Book" or self.state == "End_With_Silent" or self.state == "Error" or self.state == "Not_Found_Book_Name" or self.state == "Deny_Chitchat"):
            self.chitchat_state = False
            self.borrow_state = False
            self.return_state = False
            self.state = "Idle"
            self.main.data = self.state
            self.pub_main.publish(self.main)

    def callbackMic(self, msg):
        if msg.data == u"hướng dẫn cho tôi" or msg.data == "hướng dẫn" or msg.data == "hướng dẫn cho mình" or msg.data == "hướng dẫn đi":
            self.state = "Help"
            self.play.data = "Help.mp3"
            self.pub_play.publish(self.play)
        else:
            if msg.state == "Got" and self.state == "Idle" and msg.data == u"xin chào":
                self.state = "Busy"
                self.main.data = self.state
                self.pub_main.publish(self.main)
                self.recog.data = msg.data
                self.pub_recog.publish(self.recog)
            elif msg.state == "Got" and (self.state == "Record5" or self.state == "Record3"):
                self.state = "Busy"
                self.main.data = self.state
                self.pub_main.publish(self.main)
                self.recog.data = msg.data
                self.pub_recog.publish(self.recog)  

    def callbackChat(self, msg):
        if msg.state == "@Greeting":
            self.state = msg.state
            # self.talk.data = msg.data
            # self.pub_talk.publish(self.talk)
            self.play.data = "Ask_Book.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@Borrow_Book":
            self.borrow_state = True
            self.return_state = False
            self.chitchat_state = False
            self.state = msg.state
            self.play.data = "Ask_Book.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@Book_With_Name":
            self.borrow_state = True
            self.state = "Ask_Name"
            self.play.data = "Ask_Name.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@Check_Borrow":
            self.borrow_state = True
            self.return_state = False
            self.chitchat_state = False
            list_name = msg.data.split("%")
            self.name = list_name[0]
            self.book_name = list_name[1]
            self.author, self.amount, self.column, self.row = checkBook(self.book_name)
            self.state = msg.state
            # self.talk.data = u"tên bạn là {}".format(self.name) + " và hiện taị bạn muốn mượn sách {}. bạn xác nhận lại thông tin giúp mình được không?".format(self.book_name)
            # self.pub_talk.publish(self.talk)
            self.play.data = "Ask_Book.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@Check_Return":
            self.borrow_state = False
            self.return_state = True
            self.chitchat_state = False
            self.state = msg.state
            list_name = msg.data.split("%")
            self.name = list_name[0]
            self.book_name = list_name[1]
            self.talk.data = u"tên bạn là {}".format(self.name) + " và hiện tại bạn muốn trả sách {}. bạn xác nhận lại thông tin giúp mình được không?".format(self.book_name)
            self.pub_talk.publish(self.talk)
        elif msg.state == "@Correct":
            if self.return_state == True:
                self.state = "End_Return_Book"
                is_book, is_another_book, book_list, self.author = checkName(self.name, self.book_name)
                if is_book == True:
                    returnBook(self.name, self.book_name, self.author)
                    self.talk.data = "dữ liệu trả sách của bạn đã được ghi nhận."
                    if is_another_book == True:
                        self.talk.data = self.talk.data + " mình xin nhắc thêm, ngoài cuốn {}".format(self.book_name) + ", hiện tại bạn đang mượn thêm sách {}. bạn nhớ trả sách đúng thời hạn.".format(book_list)
                else:
                    self.talk.data = "cuốn {} bạn đang muốn trả không có trong dữ liệu mượn sách của bạn. vui lòng kiểm tra lại.".format(self.book_name)
                self.talk.data = self.talk.data + " cảm ơn bạn đã ghé thăm thư viện."
                self.pub_talk.publish(self.talk)
            elif self.borrow_state == True:
                if self.amount == 0:
                    self.state = "Out_Of_Book"
                    self.play.data = "Out_Of_Book.mp3"
                    self.pub_play.publish(self.play)
                elif self.amount == -1:
                    self.state = "Not_Found_Book_Name"
                    self.talk.data = "xin lỗi, cuốn {} bạn yêu cầu hiện không nằm trong dữ liệu sách của thư viện hoặc có thể có chút nhầm lẫn về tên sách. nếu đúng là có sự nhầm lẫn, mong bạn hãy tương tác lại từ đầu với mình. cảm ơn.".format(self.book_name)
                    self.pub_talk.publish(self.talk)
                else:
                    self.state = "Correct_Borrow_Book"
                    # self.talk.data = "sách {} bạn đang muốn tìm ".format(self.book_name) + "của tác giả {}".format(self.author) + " hiện còn {}".format(self.amount) + " quyển, vị trí ở kệ số {}".format(self.column) + ", hàng {}.".format(self.row)
                    # self.pub_talk.publish(self.talk)
                    self.play.data = "Ask_Book.mp3"
                    self.pub_play.publish(self.play)
        elif msg.state == "@Affirm_Take_Book":
            if self.chitchat_state == True:
                self.state = "Affirm_Chitchat"
                self.play.data = "Ask_Common.mp3"
                self.pub_play.publish(self.play)
            elif self.borrow_state == True:
                self.state = "Wait_Me"
                self.play.data = "Wait_Me.mp3"
                self.pub_play.publish(self.play)
        elif msg.state == "@Deny_Take_Book":
            if self.chitchat_state == True:
                self.state = "Deny_Chitchat"
                self.play.data = "End_Common.mp3"
                self.pub_play.publish(self.play)
            elif self.borrow_state == True:
                self.state = "Good_Luck"
                self.talk.data = u"mình xin nhắc lại, sách {} bạn đang muốn tìm ".format(self.book_name) + "của tác giả {}".format(self.author) + " hiện còn {}".format(self.amount) + " quyển, vị trí ở kệ số {}".format(self.column) + ", hàng {}.".format(self.row)
                self.pub_talk.publish(self.talk)
        elif msg.state == "@Chit_Chat":
            self.borrow_state = False
            self.return_state = False
            self.chitchat_state = True
            self.state = msg.state
            self.talk.data = msg.data
            self.pub_talk.publish(self.talk)
        elif msg.state == "Not_Ask":
            self.state = "End_Chitchat"
            self.play.data = "End_Chitchat.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@Return_Book":
            self.borrow_state = False
            self.return_state = True
            self.chitchat_state = False
            self.state = msg.state
            self.play.data = "Ask_Book_Return.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@aboutrobot":
            self.borrow_state = False
            self.return_state = False
            self.chitchat_state = True
            self.state = msg.state
            self.play.data = "About_Robot.mp3"
            self.pub_play.publish(self.play)
        elif msg.state == "@rephrase":
            self.state = msg.state
            self.talk.data = msg.data
            self.pub_talk.publish(self.talk)

    def spin(self):     
        while not rospy.is_shutdown():
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    StateMachine = StateMachine()
    StateMachine.spin()