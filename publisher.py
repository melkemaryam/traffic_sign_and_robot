#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from time import sleep

from ./code/predict import Predict_Net

def publish_label(self, label):

        lab = Predict_Net()
        label = lab.main_predict_net()

        pub = rospy.Publisher('ros_label', String, queue_size=10)
        rospy.init_node("ros_label", anonymous=True)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            pub.publish(label)
            rospy.loginfo("Please turn %s", label)
            rate.sleep()



if __name__ == "__main__":
    try:
        self.publish_label()
    except rospy.ROSInterruptException:
        pass