#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from time import sleep

from code.predict import Predict_Net

def publish_label():

        lab = Predict_Net() # create object for prediction
        
        rospy.init_node("ros_label", anonymous=True) # initialise ros node
        pub = rospy.Publisher('ros_label', Int16, queue_size=1) # create publisher object
        rate = rospy.Rate(5) # pause

        label = Int16() # create message object

        while not rospy.is_shutdown():

            result = lab.main_predict_net() # get label of prediction

            if (result == "Turn right"):
                label = 1 # return 1 if robot should turn right

            if (result == "Turn left"):
                label = 2 # return 2 if robot should turn left

            pub.publish(label) # publish label
            rospy.loginfo("Please %s", result) # show label in Terminal
            rate.sleep() # pause



if __name__ == "__main__":
    try:
        publish_label()
    except rospy.ROSInterruptException:
        pass