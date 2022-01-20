# How to run ROS

Please install ROS first.

## 1. Create Package

* source the ROS environment with `source ~/.bashrc`
* check for any updates with `rosdep update`
* go into your ROS src directory with `cd <catkin_ws/src>`
* create a package with `catkin_create_pkg test_me std_msgs rospy roscpp`
    * where `catkin_create_pkg` is the command,
    * `test_me` is the package name,
    * `std_msgs`, `rospy`, and `roscpp` are the dependencies
* go one step back within the directory with `cd ..`
* build the packages within the workspace with `catkin_make`
* source `source ~/.bashrc` again

## 2. Create new files

* Go to your ROS environment and create a new file inside the created package: `cd ~/catkin_ws/src/test_me/src`
* After creating the file, make it executable with: `chmod +x file.py`
* repeat the last step

## 3. Prepare file to run

* go back to the general ROS enviornment `cd ~/catkin_ws`
* run `catkin_make`
* type `cd ..`

## 4. Edit bash file

* run `nano ~/.bashrc`

The bash file must read something like this:
```
source /opt/ros/noetic/setup.bash
source /home/hannah/catkin_ws/devel/setup.bash
```
## 5. Run script

### 1. Roscore

* open a new terminal window
* run `roscore`


### 2. Run the Publisher

* run `source ~/.bashrc`
* identify the publisher file
* run `rosrun` + package + file: `rosrun ros_basics_tutorials talker.py`

### 3. Run the Subscriber

* open another terminal window
* identify the subscriber file
* run `rosrun` + package + file: `rosrun ros_basics_tutorials listener.py`



### Create Virtual Environment on Ubuntu

* run `sudo pip3 install virtualenv`
* create environment with `virtualenv <name> -p python3`
* start the environment with `source <name>/bin/activate`
* stop environment with `deactivate`