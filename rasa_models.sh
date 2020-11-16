cd ~/Librarian_ws/src/lib_chatbot/rasa
. ~/Librarian_ws/venv/bin/activate
rasa run -m models --enable-api --port 5002 --cors "*"
