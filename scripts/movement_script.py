#!/usr/bin/env python3
import rospy
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os


class Turtlebot:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('emotion_based_movement')

        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)  

        #Array of emotions movesments (duration,linar_velocity, angular_velocityt)

        self.emotion = {
            "joyful" : [( 2, 0.4, 2), (2, 0.4, 2), (1, 0.6, 0)],
            "sad" : [(1.5,-0.2,0.55),(1.5,-0.2,-0.6)],
            "anger":[( 5, 0.1, -1.0)],
            "fear": [( 1.5, -0.7, -0.6)],
            "confident":[(2, 0.3, 0.3),(1.25, 0.2, 0.9),(1.25, 0.2, -0.9),(1, 0.5, 0)],
            "happy": [(5, 0.4, -1.5)],
            "frustrated":[(5, 0.4, 1.5)],
            "tired":[(5, 0.1, 0)],
            "neutral":[],
        }

        
    def autonomousMovement(duration, linear_vel, angular_vel):

        move_cmd = Twist()
        move_cmd.linear.x = linear_vel
        move_cmd.angular.z = angular_vel

        start_time = rospy.get_time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and (rospy.get_time() - start_time) < duration:
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        # Stop the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_vel_pub.publish(move_cmd)
        # rospy.loginfo(f"'{name}' movement completed.")
    
    def executeEmotion(self,emotion):
        if emotion in self.emotion:
            rospy.loginfo(f"Executing {emotion} script")
            for duration,linear_vel,angular_vel in self.emotions[emotion]:
                autonomousMovement(duration,linear_vel,angular_vel)
        else:
            rospy.loginfo("Unknown emotion entered. Please try again")


def main():
    bot = Turtlebot()
    while not rospy.is_shutdown():
        print("Available Emotions: ", bot.emotion.keys())
        emotion = input("Enter an emotion: ").strip().lower()
        if emotion == "exit":
            rospy.loginfo("Exiting...")
            sys.exit()
        



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass