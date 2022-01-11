# TODO
* Translate relative point on pgm to coordinate frame
* Relative file path for images
* Camera move subscriber
* Map image to map location translation

# Requirements
```shell
sudo apt-get install python3-pyqt5
sudo apt-get install qtcreator pyqt5-dev-tools
sudo apt-get install qttools5-dev-tools

cd stretch_gui
pip3 install -r requirements.txt

# ROS Noetic must be installed. Instructions for this are beyond scope.
```

# Editing GUI Interface
```shell
cd stretch_uml/scripts/gui/
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
rosrun stretch_uml stretch_main_window.py
# Shell 3
rosrun stretch_uml map_subscriber
```

### Building Docs
```shell
$ cd stretch_uml/docs
$ make clean # If needed
$ make html
$ xdg-open build/html/index.html # Open it in a browser
```

### Updating Docs
```shell
$ cd stretch_uml/docs
$ sphinx-apidoc ../scripts/PACKAGENAME -o source/modules
$ cd source/modules
$ vim modules.rst
# Add the new .rst files name to the modules.rst file as shown
$ cd ..
$ vim conf.py
# append the syspath for any new module added as shown
```

* Warning about duplicate contents is fine as long as it looks correct
* Warning about document or segment not beginning with a transition is fine as long as it looks correct

### Docs Initial Setup
```shell
$ cd stretch_uml
$ mkdir docs
$ cd docs
$ sphinx-quickstart
$ mkdir source
$ cd source
$ mkdir modules
$ sphinx-apidoc ../../scripts/PACKAGENAME -o modules
```
* Add required modules to the conf.py file
* Edit conf.py to support .md
* Change theme to desired and set code highlight color
* Rerun apidoc and adjust modules.py by hand when updating
* Warning about duplicate contents is fine as long as it looks correct
