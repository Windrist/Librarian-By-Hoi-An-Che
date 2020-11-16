gnome-terminal -x sh -c ". ~/Librarian_ws/rasa_models.sh; bash"
gnome-terminal -x sh -c ". ~/Librarian_ws/rasa_actions.sh; bash"
sleep 2
gnome-terminal -x sh -c ". ~/Librarian_ws/venv/bin/activate; roslaunch lib_bringup runAll.launch; bash"

