cd ~/Workspace/Librarian_ws/src/Librarian-By-Hoi-An-Che/lib_chatbot/rasa
. ~/Workspace/Librarian_ws/venv/bin/activate
rasa run -m models --enable-api --port 5002 --cors "*"
