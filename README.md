Librarian-Robot
=
Librarian Robot that help you borrow book, return book and ask something else!

## Core Feature:

```bash
Librarian ChatBot with Power Support of Rasa
Speech to Text by Google Cloud
Text to Speech by FPT AI
Manipulator with 3 DOF
SLAM and Navigation with Lidar
```

## Requirements
- Ubuntu 16.04 / 18.04 / 20.04
- ROS - Robot Operating System
- Python 3

## Installation
```bash
mkdir Librarian_ws && cd Librarian_ws && mkdir src
python3 -m venv venv
. venv/bin/activate
cd src && git clone https://github.com/Windrist/Librarian-By-Hoi-An-Che
cd Librarian-By-Hoi-An-Che
. setup.sh
cd ../../../.. && catkin_make
```

## Usage

#### Run All:
```bash
cd ~/Librarian_ws/src/lib_run
. runAll.sh
```
#### Run Only ChatBot:
```bash
cd ~/Librarian_ws/src/lib_run
. runChatBot.sh
```

## Credits
Project Made By Hoi An Che-K63R, consists of:

- Tran Huu Quoc Dong - [Project Manager]
- Bui Duy Nam - [Main Coder]
- Hoang Quoc Anh - [ChatBot Coder]
- Dam Phuong Nam - [Designer]
- Do Tuan Anh - [Designer]
- Duong Thi Thuy Ngan - [Control Coder]
- Ngo Thi Ngoc Quyen - [Control Coder]
- Pham Quang Hung - [Control Coder]
- Dang Van Hieu - [Designer]
- Nguyen Ba Chung - [Control Coder]

