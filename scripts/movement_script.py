#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os


class Turtlebot:
    def __init__(self):
        # Initialize ROS Node
        if not rospy.core.is_initialized():
            rospy.init_node('emotion_based_movement', anonymous=True)


        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Array of emotions movesments (duration,linar_velocity, angular_velocityt)
        self.emotion = {
            "excited" : [( 2, 0.4, 2), (2, 0.4, 2), (1, 0.6, 0)],
            "sad" : [(1.5,-0.2,0.55),(1.5,-0.2,-0.6)],
            "anger":[( 5, 0.1, -1.0)],
            "fear": [( 1.5, -0.7, -0.6)],
            "confident":[(2, 0.3, 0.3),(1.25, 0.2, 0.9),(1.25, 0.2, -0.9),(1, 0.5, 0)],
            "happy": [(5, 0.4, -1.5)],
            "frustrated":[(5, 0.4, 1.5)],
            "tired":[(5, 0.1, 0)],
            "neutral":[],
        }
        self.synonyms = {
            "excited": ["excited", "eager", "enthusiastic"],
            "sad": ["sad", "unhappy", "down", "depressed"],
            "anger": ["angry", "mad", "irritated", "frustrated"],
            "fear": ["fearful", "scared", "terrified", "anxious"],
            "confident": ["confident", "secure", "assured"],
            "happy": ["happy", "joyful", "content", "pleased"],
            "frustrated": ["frustrated", "annoyed", "irritated"],
            "tired": ["tired", "exhausted", "fatigued"],
            "neutral": ["neutral", "calm", "indifferent"]
        }

        

    # Defintion the movement interaction with the script and ROS using (Duration, Linear velocity and angular velocity)
    def autonomousMovement(self,duration, linear_vel, angular_vel):
        move_cmd = Twist()
        move_cmd.linear.x = linear_vel
        move_cmd.angular.z = angular_vel

        start_time = rospy.get_time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and (rospy.get_time() - start_time) < duration:
            self.cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        # Stop the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        # rospy.loginfo(f"'{name}' movement completed.")
    
    def executeEmotion(self, emotion):
        # Check if emotion or any of its synonyms exist
        for key, value in self.synonyms.items():
            if emotion in value:
                emotion = key  # Map to the canonical emotion key
                rospy.loginfo(f"Executing {emotion} movement script")
                if self.emotion[emotion]:  # Check for non-empty list
                    for duration, linear_vel, angular_vel in self.emotion[emotion]:
                        self.autonomousMovement(duration, linear_vel, angular_vel)
                else:
                    rospy.loginfo(f"Neutral emotion: No movement.")
                break
        else:
            rospy.loginfo("Unknown emotion entered. Please try again.")



def main():
    bot = Turtlebot()
    while not rospy.is_shutdown():
        print("Available Emotions: ", bot.emotion.keys())
        emotion = input("Enter an emotion: ").strip().lower()
        bot.executeEmotion(emotion)
        if emotion == "exit":
            rospy.loginfo("Exiting...")
            sys.exit()
        

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass