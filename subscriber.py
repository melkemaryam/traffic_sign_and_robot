#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from time import sleep

# Create object to be published
result = Float64()
direction = Float64()

# Function to do the calculations
def callback(msg):
    
    result.data = msg
    direction = msg.data

    if (direction == 1.0):
        current_direction = "right"

    if (direction == 2.0):
        current_direction = "left"

    # Log the result with ros
    rospy.loginfo(rospy.get_caller_id() + "I am turning %s", current_direction)


def main():

    rospy.init_node("arduino_reaction", anonymous=True)     
 
    sub = rospy.Subscriber("arduino_reaction", Float64, callback)
    rate = rospy.Rate(1)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass