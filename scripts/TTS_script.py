#!/usr/bin/env python3

import rospy
import os
from sound_play.libsoundplay import SoundClient

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
        
        # Stop any currently playing sound before starting a new one
        rospy.sleep(1)
        self.sound_client.stopAll()
        rospy.loginfo("Stopping current sound.")

        # Play the new sound
        self.sound_client.say(text, volume=1.0, pitch=1.5, speed=100)

    

# Only run if executed as a script
if __name__ == "__main__":
    tts = TextToSpeech()
    rospy.loginfo("TTS Node is running...")
    tts.speak("Text-to-speech module is ready.")
    rospy.spin()

