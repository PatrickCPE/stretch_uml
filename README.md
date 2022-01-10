# TODO
Look at TODOs in code
* Implement subscriber that updates the local map .pgm file every # seconds
* Translate relative point on pgm to coordinate frame
* Image Selection Dot Code
* Replace Default Images with Loading/Please Wait
* Wrap up integration of all the controls
* Fix text colors where needed
* Draw a dot for strech on the map

# Notes
* Map image is called map.pgm and it is in the scripts folder of the project

# Requirements
```shell
sudo apt-get install python3-pyqt5
sudo apt-get install qtcreator pyqt5-dev-tools
sudo apt-get install qttools5-dev-tools

cd stretch_gui
pip3 install -r requirements.txt

# ROS Noetic must be installed. Instructions for this are beyond scope.
```

# Editing Code and Interface
```shell
cd stretch_gui
# Launch QT Designer
designer
# Open .ui file. Save edits when complete
pyuic5 -x stretch.ui -o stretch_ui_main_window.py
```

# Usage
```shell
# Shell 1
roscore
# Shell 2
python stretch_main_window.py
```