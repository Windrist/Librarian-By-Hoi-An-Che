gnome-terminal -x sh -c ". ~/Workspace/Librarian_ws/src/Librarian-By-Hoi-An-Che/lib_run/scripts/rasa_models.sh; bash"
gnome-terminal -x sh -c ". ~/Workspace/Librarian_ws/src/Librarian-By-Hoi-An-Che/lib_run/scripts/rasa_actions.sh; bash"
sleep 10
gnome-terminal -x sh -c ". ~/Workspace/Librarian_ws/venv/bin/activate; roslaunch lib_bringup runAll.launch; bash"

