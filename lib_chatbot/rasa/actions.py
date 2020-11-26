# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/core/actions/#custom-actions/


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List, Union 

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet, AllSlotsReset, FollowupAction 
from rasa_sdk.forms import FormAction 
import random
import requests
import rospy
from rasa_sdk.events import UserUtteranceReverted
from bs4 import BeautifulSoup
import urllib

class ActionAskBook(FormAction):
    def name(self) -> Text:
        return "form_ask_book"
    
    @staticmethod
    def required_slots(tracker: Tracker) -> List[Text]:
        SlotSet(key = 'state', value = 'borrow_book')
        return ["book_name","user_name"]

    # def slot_mappings(self) ->  Dict[Text, Union[Dict, List[Dict]]]:
    #     return {
    #         "user_name": [self.from_entity(entity="user_name", intent=["give_name"] ), self.from_text()],
    #         "book_name": [self.from_entity(entity="book_name", intent=["ask_book"] ), self.from_text()],
    #     }
    # def request_next_slot(self, dispatcher, tracker, domain):
    #     intent = tracker.latest_message['intent'].get('name')
    #     if intent == 'deny':
    #         return self.deactivate()

    def submit(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        book_name = tracker.get_slot("book_name")
        user_name = tracker.get_slot("user_name")
        if book_name == None or user_name == None:
            return self.deactivate()
        dispatcher.utter_message(text = "@Check_Borrow-{}%{}".format(user_name, book_name))
        return []

class ActionReturnBook(FormAction):
    def name(self) -> Text:
        return "form_return_book"
    
    @staticmethod
    def required_slots(tracker: Tracker) -> List[Text]:
        SlotSet(key = 'state', value = 'return_book')
        return ["book_name","user_name"]

    def submit(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        book_name = tracker.get_slot("book_name")
        user_name = tracker.get_slot("user_name")
        if book_name == None or user_name == None:
            return self.deactivate()
        dispatcher.utter_message(text = "@Check_Return-{}%{}".format(user_name, book_name))
        return []
        
class ActionFallback(Action):
    def name(self) -> Text:
        return "action_fallback"
    
    async def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(template="utter_please_rephrase")
        return[UserUtteranceReverted()]

class ActionResolve(Action):
    def name(self) -> Text:
        return "action_resolve"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        confirm = tracker.latest_message['intent'].get('name')
        book_name = tracker.get_slot("book_name")
        user_name = tracker.get_slot("user_name")
        state = tracker.get_slot("state")

        if state == '@Borrow_Book-':
            if book_name == None or user_name == None:
                return [FollowupAction("form_ask_book")]
            if confirm == 'wrong_book_name':
                return [SlotSet(key = 'book_name', value = None), FollowupAction("form_ask_book")]
            elif confirm == 'wrong_user_name':
                return [SlotSet(key = 'user_name', value = None), FollowupAction("form_ask_book")]
            elif confirm == 'wrong':
                return [AllSlotsReset(), FollowupAction("form_ask_book")]
            elif confirm == 'correct':
                dispatcher.utter_message(text = "@Correct-")
                return [AllSlotsReset()]

        elif state == '@Return_Book-':
            if book_name == None or user_name == None:
                return [FollowupAction("form_return_book")]
            if confirm == 'wrong_book_name':
                return [SlotSet(key = 'book_name', value = None), FollowupAction("form_return_book")]
            elif confirm == 'wrong_user_name':
                return [SlotSet(key = 'user_name', value = None), FollowupAction("form_return_book")]
            elif confirm == 'wrong':
                return [AllSlotsReset(), FollowupAction("form_return_book")]
            elif confirm == 'correct':
                dispatcher.utter_message(text = "@Correct-")
                return [AllSlotsReset()]

        return []

class ActionTakeBook(Action):
    def name(self) -> Text:
        return "action_take_book"
    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        confirm = tracker.latest_message['intent'].get('name')

        if confirm == 'deny':
            dispatcher.utter_message(text = "@Deny_Take_Book-")
        elif confirm == 'affirm':
            dispatcher.utter_message(text = "@Affirm_Take_Book-")
        return []

class ActionSetState(Action):
    def name(self) -> Text:
        return "action_set_state"

    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        pre_intent = tracker.latest_message['intent'].get('name')
        if pre_intent == 'ask_book':
            return [SlotSet(key = 'state', value = '@Borrow_Book-')]
        elif pre_intent == 'return_books':
            return [SlotSet(key = 'state', value = '@Return_Book-')]
        return []
# class ActionAskConfirm(Action):
#     def name(self) -> Text:
#         return "action_ask_confirm"
    
#     def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
#         dispatcher.utter_message(text = "bạn xác nhận lại giúp mình với")
#         return []
        

class ActionAskWeather(Action):
    def name(self) -> Text:
        return "action_ask_weather"
    
    def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, 
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

            url = 'https://nchmf.gov.vn/Kttvsite/vi-VN/1/tp-ha-noi-w28.html'
            page = urllib.request.urlopen(url)
            soup = BeautifulSoup(page, 'html.parser')

            status = soup.find('section', class_='row-wrp').find_all('div',{'class': 'uk-width-1-4'})
            value = soup.find('section', class_='row-wrp').find_all('div',{'class': 'uk-width-3-4'})
            info = []
            for i in range(len(status)):
                value[i]= value[i].get_text()
            dispatcher.utter_message(text = "@Chit_Chat- đây là thông tin về thời tiết hôm nay nhiệt độ {}, bầu trời {}, độ ẩm {} phần trăm, hướng {}  ".format(value[0], value[1], value[2], value[3]))
            return []

