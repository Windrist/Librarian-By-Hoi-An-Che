

## about robot
* ask_infor_robot
  - utter_about_robot
* goodbye
  - utter_goodbye

## borrow book
* greet
  - utter_greet
* ask_book
  - action_set_state
  - form_ask_book
* correct
  - action_resolve
* affirm
  - action_take_book
  
## borrow book 2
* ask_book
  - action_set_state
  - form_ask_book
* wrong_user_name
  - action_resolve
* wrong_book_name
  - action_resolve
* correct
  - action_resolve
* affirm
  - action_take_book

## borrow book 3
* ask_book
  - action_set_state
  - form_ask_book
* wrong
  - utter_ask_what_wrong

## borrow book 4
* ask_book
  - action_set_state
  - form_ask_book
* wrong_user_name
  - action_resolve
* wrong_book_name
  - action_resolve
* correct
  - action_resolve
* deny
  - action_take_book

## return book 1
* return_books
  - action_set_state
  - form_return_book
* correct
  - action_resolve

## return book 2
* return_books
  - action_set_state
  - form_return_book
* wrong_user_name
  - action_resolve
* wrong_book_name
  - action_resolve
* correct
  - action_resolve

## return book 3
* return_books
  - action_set_state
  - form_return_book
* wrong
  - utter_ask_what_wrong

## thank robot
* thank_robot
  - utter_thank_robot

## out of scope
* out_of_scope
  - action_fallback

## ask weather
* ask_weather_today
  - action_ask_weather

# need help
* need_help
  - action_need_help