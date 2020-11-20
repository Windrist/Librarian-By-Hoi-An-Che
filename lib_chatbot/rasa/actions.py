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
                dispatcher.utter_message(text = "Correct-")
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
        
# class ActionFacilitiesSearch(Action):
#     def name(self) -> Text:
#         return "action_facilities_search"
    
#     def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, 
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
#             listener()
#             facility = tracker.get_slot("facilities")
#             address = "Ngõ 32, phố Chùa Hà, Cầu Giấy, Hà Nội"
#             dispatcher.utter_message(text = "Đây là địa chỉ của {}:{}".format(facility, address))
#             return []

# class ActionAskWeather(Action):
#     def name(self) -> Text:
#         return "action_ask_weather"
    
#     def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, 
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

#             url = 'https://nchmf.gov.vn/Kttvsite/vi-VN/1/tp-ha-noi-w28.html'
#             page = urllib.request.urlopen(url)
#             soup = BeautifulSoup(page, 'html.parser')

#             status = soup.find('section', class_='row-wrp').find_all('div',{'class': 'uk-width-1-4'})
#             value = soup.find('section', class_='row-wrp').find_all('div',{'class': 'uk-width-3-4'})
#             info = []
#             for i in range(len(status)):
#                 status[i]= status[i].get_text()
#                 value[i]= value[i].get_text()
#                 info.append(status[i]+value[i])

#             weather = str(tracker.get_slot("weather"))
#             day_time = str(tracker.get_slot("time"))
#             if day_time=='ngày mai':
#                  dispatcher.utter_message(text = "{} {} thì {}".format(weather, day_time, 'Bot không biết :))'))
#             elif day_time=='hôm nay':
#                 dispatcher.utter_message(text = "Đây là thông tin về thời tiết hôm nay:")
#                 for i in info:
#                     dispatcher.utter_message(text = i)
#             else:
#                 dispatcher.utter_message(text = "không biết:{}".format(day_time))
#                 dispatcher.utter_message(text = "hỏi câu khác đi")
#             return []

# class ActionAskKQXS(Action):
#     def name(self) -> Text:
#         return "action_ask_kqxs"
    
#     def run(self, dispatcher: CollectingDispatcher, tracker: Tracker,
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
#             url = 'https://xoso.me/xsmb-sxmb-xstd-xshn-kqxsmb-ket-qua-xo-so-mien-bac.html'
#             page = urllib.request.urlopen(url)
#             soup = BeautifulSoup(page, 'html.parser')

#             status = soup.find('section', class_ = 'content main clearfix').find_all('td', {'class': 'number'}, {'colspan': '12'})
#             dispatcher.utter_message(text = 'Giải đặc biệt : {}'.format(status[0].get_text()))

# class WorkshopForm(FormAction):
#     def name(self):
#         return "workshop_form"
    
#     @staticmethod
#     def required_slots(tracker: Tracker) -> List[Text]:
#         return ["user_name", "phone_number", "attendee_no"]

#     def submit(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Dict]:
#         dispatcher.utter_message("Thanks, great job !")
#         user_name = tracker.get_slot("user_name")
#         phone_number = tracker.get_slot("phone_number")
#         attendee_no = tracker.get_slot("attendee_no")
#         dispatcher.utter_message("{} {} {} {}".format(user_name, phone_number, attendee_no))
        # return []
#
#
# class ActionHelloWorld(Action):
#
#     def name(self) -> Text:
#         return "action_hello_world"
#
#     def run(self, dispatcher: CollectingDispatcher,
#             tracker: Tracker,
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
#
#         dispatcher.utter_message(text="Hello World!")
#
#         return []
