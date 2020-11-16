gnome-terminal -x sh -c ". ~/Librarian_ws/src/Librarian-By-Hoi-An-Che/lib_run/scripts/rasa_models.sh; bash"
gnome-terminal -x sh -c ". ~/Librarian_ws/src/Librarian-By-Hoi-An-Che/lib_run/scripts/rasa_actions.sh; bash"
sleep 2
gnome-terminal -x sh -c ". ~/Librarian_ws/venv/bin/activate; roslaunch lib_bringup chatBot.launch; bash"

