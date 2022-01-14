#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from time import sleep

# Create object to be published
result = String()

# Function to do the calculations
def callback(msg):
    
    result.data = msg

    # Log the result with ros
    rospy.loginfo(rospy.get_caller_id() + "I am going to %s", msg.data)


def main():
    rospy.init_node("arduino_reaction", anonymous=True)     
 
    sub = rospy.Subscriber("arduino_reaction", String, callback)
    rate = rospy.Rate(1)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass