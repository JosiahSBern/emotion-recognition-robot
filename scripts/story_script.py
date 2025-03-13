#!/usr/bin/env python3
import tkinter as tk
import random
import rospy
from movement_script import Turtlebot
from TTS_script import TextToSpeech

class Story:
    def __init__(self, text, emotions, correct_emotion):
        self.text = text
        self.emotions = emotions
        self.correct_emotion = correct_emotion

# Initialize ROS and robot components
rospy.init_node('storytelling_node', anonymous=True)
robot = Turtlebot()
tts = TextToSpeech()

# Emotion colors mapping
emotion_colors = {
    "Joyful": "#e0c158",      # Yellow
    "Sad": "#6da7d1",         # Blue
    "Anger": "#ce8271",       # Reddish/Pink
    "Fear": "#59b59d",        # Turquoise/Dark Green
    "Confident": "#d3e7b3",   # Light Green
    "Happy": "#edf5e0",       # Very Light Green
    "Frustrated": "#daa194",  # Light Red/Pink
    "Tired": "#cad4e1",       # Pale Blue
    "Neutral": "#ffffff"      # White
}

# List of stories
stories = [
    Story(
        text="Today is my birthday, I am going to have a big birthday party with my friends. I am looking forward to it, and I am very…",
        emotions=["Joyful", "Confident", "Sad"],
        correct_emotion="Joyful"
    ),
    Story(
        text="Today, I am going to the aquarium with my sister. I look forward to seeing so many animals. I feel very...",
        emotions=["Frustrated", "Joyful", "Neutral"],
        correct_emotion="Joyful"
    )
]

# Function to shuffle stories
def shuffle_stories(stories):
    random.shuffle(stories)
    return stories

# Text-to-speech function
def text_audio(text):
    tts.speak(text)  # Use the text-to-speech module
    print(text)  # Print to console for debugging

# Score tracking
correct_ans = 0
wrong_ans = 0

# Emotion-button click function
def emotion_button(selected_emotion, correct_emotion, score_label):
    global correct_ans, wrong_ans
    if selected_emotion == correct_emotion:
        correct_ans += 1
        tts.speak(f"Yes, I feel {selected_emotion}")
        robot.executeEmotion(selected_emotion.lower())  # Make the robot respond
    else:
        wrong_ans += 1
        tts.speak("That is not the correct emotion. Try again.")

    total = correct_ans + wrong_ans
    score_label.config(text=f"Correct: {correct_ans}  Wrong: {wrong_ans}  Score: {correct_ans}/{total}")

# Navigation function
def navigate(window, new_index):
    show_story(window, new_index)

# Show story function
def show_story(window, story_index):

    # Clear previous widgets
    for widget in window.winfo_children():
        widget.destroy()

    # Get the current story object
    story = stories[story_index]

    # Score label
    score_label = tk.Label(window, text=f"Correct: {correct_ans}  Wrong: {wrong_ans}  Score: {correct_ans}/{correct_ans + wrong_ans}",
                           font=("Arial", 16, "bold"))
    score_label.pack(pady=10)

    # Story frame for repeat audio and story text
    story_frame = tk.Frame(window)
    story_frame.pack(anchor="center", pady=100)

    # Replay audio button
    repeat = tk.Button(
        story_frame,
        text="Repeat",
        font=("Arial", 15, "bold"),
        command=lambda: text_audio(story.text)
    )
    repeat.grid(row=0, column=0, padx=20)

    # Story label
    story_label = tk.Label(
        story_frame,
        text=story.text,
        font=("Arial", 24, "bold"),
        wraplength=700,
        justify="center"
    )
    story_label.grid(row=0, column=1, padx=20)

    # Speak the story text
    text_audio(story.text)

    # Emotion buttons
    emotion_frame = tk.Frame(window)
    emotion_frame.pack(anchor="center", pady=50)

    for i, emotion in enumerate(story.emotions):
        color = emotion_colors[emotion]  # Looks up correct color
        tk.Button(
            emotion_frame,
            text=emotion,
            command=lambda emotion=emotion: emotion_button(emotion, story.correct_emotion, score_label),
            activebackground=color,
            background=color,
            width=25,
            height=3,
            font=("Arial", 14, "bold")
        ).grid(row=0, column=i, padx=20)

    # Navigation buttons
    nav_frame = tk.Frame(window)
    nav_frame.pack(anchor="center", pady=50)

    # Previous button
    prev = tk.Button(
        nav_frame,
        text="Previous",
        command=lambda: navigate(window, story_index - 1),
        state="normal" if story_index > 0 else "disabled",
        background="#cfcfcf",
        font=("Arial", 14, "bold"),
        width=15
    )
    prev.grid(row=0, column=0, padx=20, pady=20)

    # Next button
    next = tk.Button(
        nav_frame,
        text="Next",
        command=lambda: navigate(window, story_index + 1),
        state="normal" if story_index < len(stories) - 1 else "disabled",
        background="#cfcfcf",
        font=("Arial", 14, "bold"),
        width=15
    )
    next.grid(row=0, column=1, padx=20, pady=20)

# Initialize main window
def run_storytelling(stories):

    # Open window
    root = tk.Tk()
    root.geometry("1600x900")
    root.title("Stories Quiz")
    root.configure(bg="#f2f1ec")

    # Shuffle stories before starting
    shuffled_stories = shuffle_stories(stories)

    # Show the first story
    show_story(root, 0)
    root.mainloop()

# Reset global variables
correct_ans = 0
wrong_ans = 0

# Run the storytelling application
run_storytelling(stories)
