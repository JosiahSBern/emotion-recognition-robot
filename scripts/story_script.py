#!/usr/bin/env python3
import tkinter as tk
import random
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageTk
import json
import tkinter as tk
import random
import rospy
from movement_script import Turtlebot
from TTS_script import TextToSpeech
import os

emotion_colors = {
    "Joyful": "#e0c158",      # Yellow 
    "Sad": "#6da7d1",         # Blue 
    "Angry": "#ce8271",       # Redish/Pink 
    "Scared": "#59b59d",      # Turquoise/Darkish green 
    "Confident": "#d3e7b3",   # Light green 
    "Happy": "#edf5e0",       # Very Light Green 
    "Frustrated": "#daa194",  # Lighter redish/pink 
    "Tired": "#cad4e1",       # Paleish Blue 
    "Neutral": "#ffffff",     # White 
    "Creative": "#ff6f61",    # Coral 
    "Skeptical": "#9e9e9e",   # Grey 
    "Anxious": "#f8b400",     # Yellow-Orange 
    "Surprised": "#ffbd00",   # Bright yellow 
    "Guilty": "#b23a48",      # Dark red 
    "Confused": "#8e8e8e",    # Muted Grey 
    "Sleepy": "#9c88b4",      # Lavender 
    "Secure": "#66b3b7",      # Soft Teal 
    "Bored": "#e4b7c6",       # Soft pink 
    "Excited": "#ff5733",     # Bright Orange-Red 
    "Thankful": "#f9c74f",    # Warm yellow 
    "Mad": "#e60000",         # Bright red 
    "Lonely": "#a9a9a9",      # Dark grey 
    "Peaceful": "#a8d5ba",    # Soft green 
    "Overwhelmed": "#7f8c8d",  # Cool grey 
}



class Story:
    def __init__(self, text, emotions, correct_emotion, image_path):
        self.text = text
        self.emotions = emotions
        self.correct_emotion = correct_emotion
        self.image_path = image_path

def load_stories_from_json(filename=None):
    if filename is None:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(base_dir, "filtered_stories.json")
    
    with open(filename, 'r') as file:
        data = json.load(file)

    stories = []
    for item in data:
        image_path = item.get('image', '')
        story = Story(
            text=item['story'],
            emotions=item['options'],
            correct_emotion=item['correct_emotion'],
            image_path=image_path
        )
        stories.append(story)
    
    return stories



# Initialize ROS and robot components
rospy.init_node('storytelling_node', anonymous=True)
robot = Turtlebot()
tts = TextToSpeech()

# Initialize global variables
correct_ans = 0
wrong_ans = 0
score_label = 0
selected_emotions = []
feedback_box = None

def text_audio(text):
    print(text)

def robot_execute(emotion):
    pass

def shuffle_stories(stories_list):
    random.shuffle(stories_list)
    return stories_list

def emotion_button(selected_emotion, correct_emotion, window, story_index):
    global correct_ans, wrong_ans, feedback_box

    if feedback_box:
        feedback_box.destroy()

    selected_emotions.append(selected_emotion)

    feedback_box = tk.Label(
        window, font=("Nunito", 40, "bold"), padx=50, pady=30, relief="solid", bd=5
    )
    feedback_box.place(relx=0.5, rely=0.3, anchor="center")  

    if selected_emotion == correct_emotion:
        correct_ans += 1
        feedback_box.config(text="CORRECT! ✅", fg="green", background="white")
        robot_execute(emotion)
    else:
        wrong_ans += 1
        feedback_box.config(text="WRONG! ❌", fg="red", background="white")

    score_label.config(text=f"{correct_ans} / {correct_ans + wrong_ans}")

    window.after(1500, lambda: feedback_box.destroy())

def navigate(window, story_index):
    if 0 <= story_index < len(stories):
        show_story(window, story_index)

def restart_game(window):
    global correct_ans, wrong_ans, selected_emotions, score_label, feedback_box

    correct_ans = 0
    wrong_ans = 0
    selected_emotions = []

    for widget in window.winfo_children():
        widget.destroy()

    feedback_box = None 

    score_frame = tk.Frame(window)
    score_frame.pack(side="top", anchor="ne", padx=40, pady=20)

    score_label = tk.Label(score_frame, text=f"{correct_ans} / {correct_ans + wrong_ans}", 
                           font=("Nunito", 28, "bold"))
    score_label.pack()

    show_story(window, 0)

def show_story(window, story_index):
    global score_label, feedback_box

    for widget in window.winfo_children():
        widget.destroy()

    score_frame = tk.Frame(window)
    score_frame.pack(side="top", anchor="ne", padx=40, pady=20)

    score_label = tk.Label(score_frame, text=f"{correct_ans} / {correct_ans + wrong_ans}", 
                           font=("Nunito", 28, "bold"))
    score_label.pack()

    story = stories[story_index]

    story_label = tk.Label(window, text=story.text, font=("Nunito", 24), wraplength=1000, justify="center")
    story_label.pack()

    

    image_frame = tk.Frame(window)
    image_frame.pack(anchor="center")
    img = Image.open(story.image_path)
    img = img.resize((500, 350))
    photo = ImageTk.PhotoImage(img)
    image_label = tk.Label(image_frame, image=photo)
    image_label.image = photo
    image_label.pack()

    emotion_frame = tk.Frame(window)
    emotion_frame.pack(anchor="center", pady=20)

    for i, emotion in enumerate(story.emotions):
        color = emotion_colors[emotion]
        tk.Button(
            emotion_frame,
            text=emotion,
            command=lambda emotion=emotion: emotion_button(emotion, story.correct_emotion, window, story_index),
            activebackground=color,
            background=color,
            width=30,
            height=5,
            font=("Nunito", 14)
        ).grid(row=0, column=i, padx=20)

    nav_frame = tk.Frame(window, background="white")
    nav_frame.pack(pady=20)

    prev_button = tk.Button(
        nav_frame, text="◀",  
        command=lambda: navigate(window, story_index - 1),
        state="normal" if story_index > 0 else "disabled",
        font=("Nunito", 24),
        width=5,
        background="white",
        borderwidth=0
    )
    prev_button.grid(row=0, column=0, padx=60)

    prev_caption = tk.Label(nav_frame, text="Go back", font=("Nunito", 14), background="white")
    prev_caption.grid(row=1, column=0, padx=60, pady=2)

    next_button = tk.Button(
        nav_frame, text="▶",  
        command=lambda: navigate(window, story_index + 1),
        state="normal" if story_index < len(stories) - 1 else "disabled",
        font=("Nunito", 24),
        width=5,
        background="white",
        borderwidth=0
    )
    next_button.grid(row=0, column=1, padx=60)

    next_caption = tk.Label(nav_frame, text="Next", font=("Nunito", 14), background="white")
    next_caption.grid(row=1, column=1, padx=60, pady=2)


correct_ans = 0
wrong_ans = 0
score_label = 0

def run_storytelling():
    global stories
    stories = load_stories_from_json()

    root = tk.Tk()
    root.geometry("1440x900")
    root.title("Stories Quiz")

    stories = shuffle_stories(stories)

    show_story(root, 0)
    root.mainloop()

run_storytelling()


