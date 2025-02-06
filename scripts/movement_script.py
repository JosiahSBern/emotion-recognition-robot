#!/usr/bin/env python3
import rospy
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os

# Initialize ROS Node
rospy.init_node('emotion_based_movement')

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.sleep(1)  



def autonomousMovement(name, duration, linear_vel, angular_vel):

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
    rospy.loginfo(f"'{name}' movement completed.")



def main():

    while not rospy.is_shutdown():
        #Joyful,Sad,Anger,Fear,Confident,Happy,Frustrated,Tired,Neutral
        emotions = ["joyful","sad","anger","fear","confident","happy","frustrated","tired","neutral"]
        print("Availiable Emotions: ",emotions)
        emotion = input("Enter an emotion: ")

    
        
        if emotion in emotions:
            rospy.loginfo(f"Executing '{emotion}' movement...")
        
        else:
            rospy.loginfo("Unknown emotion entered. Please try again.")
            continue

        #Emotion Movements

        #Joyful-Movement
        if emotion == "joyful":
            try:
                for _ in range(2):  # Repeat the joyful spin multiple times
                    autonomousMovement('joyful_script', 2, 0.4, 2)  
                autonomousMovement('joyful_script', 1, 0.6, 0)
                
            except rospy.ROSInterruptException:
                pass

        #Sad-Movement
        elif emotion == "sad":
            try:
                autonomousMovement('sad_script', 1.5, -0.2, 0.55)
                autonomousMovement('sad_script', 1.5, -0.2, -0.6)
            except rospy.ROSInterruptException:
                pass
        #Anger-Movement
        elif emotion == "anger":
            try:
                autonomousMovement('anger_script', 5, 0.1, -1.0)
            except rospy.ROSInterruptException:
                pass
        #Fear-Movement
        elif emotion == "fear":
            try:
                autonomousMovement('fear_script', 2.5, -0.6, 0.4)
                autonomousMovement('fear_script', 1.5, -0.7, -0.6)
            except rospy.ROSInterruptException:
                pass
        #Confident-Movement
        elif emotion == "confident":
            try:
                autonomousMovement('confident_script', 2, 0.3, 0.3)
                autonomousMovement('confident_script', 1.25, 0.2, 0.9) 
                autonomousMovement('confident_script', 1.25, 0.2, -0.9) 
                autonomousMovement('confident_script', 1, 0.5, 0) 
            except rospy.ROSInterruptException:
                pass
        #Happy-Movement
        elif emotion == "happy":
            try:
                autonomousMovement('happy_script', 5, 0.4, -1.5) 
            except rospy.ROSInterruptException:
                pass
        #Frustrated-Movement
        elif emotion == "frustrated":
            try:
                autonomousMovement('frustrated_script', 5, 0.4, 1.5) 
            except rospy.ROSInterruptException:
                pass
        #Tired-Movement
        elif emotion == "tired":
            try:
                autonomousMovement('tired_script', 5, 0.1, 0) 
            except rospy.ROSInterruptException:
                pass
        #Neutral
        elif emotion == "neutral":
            try:
                autonomousMovement('neutral', 5, 0, 0) 
            except rospy.ROSInterruptException:
                pass
            
        elif emotion == "exit":
            speak("Goodbye")
            rospy.loginfo("Exiting...")
            sys.exit()
        



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
