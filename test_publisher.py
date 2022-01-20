#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from time import sleep

from code.predict import Predict_Net

def publish_label():

        lab = Predict_Net()
        #label = lab.main_predict_net()

        rospy.init_node("ros_label", anonymous=True)
        pub = rospy.Publisher('ros_label', Int16, queue_size=1)
        rate = rospy.Rate(1)

        label = Int16()

        while not rospy.is_shutdown():

            result = lab.main_predict_net()

            if (result == "Turn right"):
                label = 1

            if (result == "Turn left"):
                label = 2

            pub.publish(label)
            rospy.loginfo("Please %s", result)
            rate.sleep()



if __name__ == "__main__":
    try:
        publish_label()
    except rospy.ROSInterruptException:
        pass