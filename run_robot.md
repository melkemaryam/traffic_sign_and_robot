# How to run the robot

## 1. Set up ROS

The manual `run_ros.py` gives instructions on how to set up the correct ROS environment. For this purpose, please create a new package called `traffic_signs` in the same way it is described in the manual. 

## 2. Clone the GitHub repository

Type `cd src` and you should find yourself in the source folder of your new package.

Please clone the GitHub repository `traffic_sign_and_robot` with [this link.](https://github.com/melkemaryam/traffic_sign_and_robot.git)

This repository should be located within the source folder of your ROS package to make it work.

## 3. Train the neural network

Open a new Terminal window.

Following the instructions from the `run_code.py` manual, train the convolutional neural network with your preferred parameters. 

The network will be automatically trained and tested, so the prediction accuracy can be shown at the end. 

It is recommended to train the network until it achieves above 95% prediction accuracy. Once that is finished, the model is saved in the `output` folder. 

If nothing is changed here, the model is the same which will be used for the robot later on. 

## 4. Set up Arduino

The robot is build using an Arduino Nano mounted onto a car chassis. For this puprose, it is assumed that the robot is fully built and working. 

Before the robot can be connected in any way, the basic libraries need to be installed.

Please install the following packages:
`sudo apt-get install ros-<ros_version>-rosserial-arduino`
`sudo apt-get install ros-<ros_version>-rosserial`

After that, go into your `Arduino/libraries` directory and run this command:

`rosrun rosserial_arduino make_libraries.py .`

Now restart Arduino. `ros_lib` should now be shown under Examples.

## 5. Make files executable

Before starting the programme, please make all files executable.

The `robot.ino` file must be made executable through the Arduino settings.

The GitHub repository can be made executable as a whole, by running this command:

`~catkin_ws/src/traffic_signs/src $ chmod +x traffic_sign_and_robot`

Go back to the general ROS enviornment with `cd ~/catkin_ws` and run `catkin_make`. After that type `cd ..`.
Now source the bash file with `source ~/.bashrc`.

## 5. Connect the robot

Connect your robot to your device via USB and open the Arduino application with the file `robot.ino`. Make sure that the pins are all correct.

Now, upload the code to the Arduino. Please make sure to check on the chosen USB port. It could look like this: `/dev/ttyUSB0`.

The robot should now automatically start to drive forward.

## 6. Establish the ROS connection

Open a new terminal window and type: `roscore`.

Open a new terminal window and type: `rosrun rosserial_python serial_node.py /dev/ttyUSB0`.

Open a new terminal window and start the publisher. First go into the `output` directory, then type: `rosrun traffic_signs publisher.py`.

Open a new terminal window and start the subscriber by typing: `rosrun traffic_signs subscriber.py`.

## 7. Observe the programme

The publisher should now load a new traffic sign image every few seconds, predict the label and send it to the Arduino. The Arduino will then act according to the received label.

To check the predictions, you can open the `predictions_rl` folder. There, the images will chnage in correspondence with the publisher.
