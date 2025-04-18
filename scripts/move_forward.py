#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_forward():
    rospy.init_node('move_forward_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    move_cmd = Twist()
    move_cmd.linear.x = 0.2  # Forward at 0.2 m/s
    move_cmd.angular.z = 0.0

    rospy.loginfo("Moving forward...")

    start_time = rospy.Time.now().to_sec()
    duration = 5.0  # seconds

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        if current_time - start_time < duration:
            pub.publish(move_cmd)
            rate.sleep()
        else:
            break

    # Stop the robot
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Stopping.")

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
