#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
import string
from time import sleep

# Create object to be published
result = Int16()
direction = Int16()
current_direction = string

# callback function when receiving message
def callback(msg):
    
    # receive data
    result.data = msg
    direction = msg.data

    if (direction == 1):
        
        # receive direction
        current_direction = "right"
        
        # Log the result with ros
        rospy.loginfo("I am turning %s", current_direction)

    if (direction == 2):
        
        # receive direction
        current_direction = "left"
        
        # Log the result with ros
        rospy.loginfo("I am turning %s", current_direction)


def main():

    # initialise ros node
    rospy.init_node("arduino_reaction", anonymous=True)     
 
    # create subscriber object
    sub = rospy.Subscriber("arduino_reaction",Int16, callback)
    rate = rospy.Rate(1) # pause

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass