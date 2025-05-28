#!/usr/bin/env python3
import rospy
import os
import requests
from sound_play.libsoundplay import SoundClient

class TextToSpeech:
    def __init__(self, flask_url):
        if not rospy.core.is_initialized():
            rospy.init_node('text_to_speech_node', anonymous=True)

        self.flask_url = flask_url
        self.sound_client = SoundClient()
        rospy.sleep(1)

    def speak(self, text):
        """Request emotional TTS and play it using sound_play in ROS"""
        rospy.loginfo(f"[TTS] Requesting: '{text}' with emotion: {emotion}")
        
        # Prepare payload with optional emotion (you can add this to your Flask API if supported)
        payload = {
            "text": text,
        }

        try:
            # POST request to your Flask TTS server
            response = requests.post(self.flask_url, json=payload)
            response.raise_for_status()  # Raise error for HTTP error codes

            if response.status_code == 200:
                # Save audio to a temp file (could remove the duplicate save)
                with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as temp_audio:
                    temp_audio.write(response.content)
                    temp_path = temp_audio.name
                
                rospy.loginfo(f"[TTS] Saved temp audio to: {temp_path}")

                # Stop any currently playing sounds to avoid overlap
                self.sound_client.stopAll()
                rospy.sleep(0.5)  # Small pause to ensure stop is processed
                
                # Play the new TTS audio
                self.sound_client.playWave(temp_path)
                rospy.loginfo(f"[TTS] Playing audio...")

        except Exception as e:
            rospy.logerr(f"[TTS ERROR] {e}")

if __name__ == "__main__":
    flask_tts_url = "http://192.168.68.60:5000/speak"  # Replace with your Flask server URL

    tts = TextToSpeech(flask_url=flask_tts_url)
    rospy.loginfo("Emotional TTS Node is ready.")

    # Test
    tts.speak("Hi, I am excited to see you!")

    rospy.spin()
