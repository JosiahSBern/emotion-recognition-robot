#!/usr/bin/env python3

import rospy
import os
from sound_play.libsoundplay import SoundClient

class TextToSpeech:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('text_to_speech_node', anonymous=True)
        self.sound_client = SoundClient()
        rospy.sleep(1)  # Allow time for sound_play to start

    def speak(self, text):
        """Convert text to speech using ROS sound_play"""
        rospy.loginfo(f"Speaking: {text}")
        self.sound_client.say(text)
        rospy.sleep(2)  # Allow time for the text to be spoken
    

# Only run if executed as a script
if __name__ == "__main__":
    tts = TextToSpeech()
    rospy.loginfo("TTS Node is running...")
    tts.speak("Text-to-speech module is ready.")
    rospy.spin()
