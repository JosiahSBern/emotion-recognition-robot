#!/usr/bin/env python3
import tkinter as tk
import random
import rospy
from movement_script import Turtlebot
from TTS_script import TextToSpeech
import json
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageTk
from collections import defaultdict





import os
class Story:
    def __init__(self, text, emotions, correct_emotion,image_path):
        self.text = text
        self.emotions = emotions
        self.correct_emotion = correct_emotion
        self.image_path = image_path


# Initialize ROS and robot components
rospy.init_node('teaching_node', anonymous=True)
robot = Turtlebot()
tts = TextToSpeech()


# Emotion colors mapping
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
    "Lonely": "#a9a9a9",      # Dark grey (
    "Peaceful": "#a8d5ba",    # Soft green 
    "Overwhelmed": "#7f8c8d"  # Cool grey 
}


def load_stories_from_json():
    script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script directory
    filename = os.path.join(script_dir, "filtered_stories.json")  # Full path to JSON file
    
    # Check if file exists before opening
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found!")
        exit(1)
    
    with open(filename, 'r') as file:
        data = json.load(file)  # Load JSON

    # Process the JSON data into Story objects
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

# Load stories
stories = load_stories_from_json()


# Function to shuffle stories
def shuffle_stories(stories):
    random.shuffle(stories)
    return stories

# Text-to-speech function
def text_audio(text):
    tts.speak(text)  # Use the text-to-speech module
    print(text)  # Print to console for debugging


def flip_flashcard(flashcard_text, flashcard_image, flashcard_frame, story, is_front):

    if is_front[0]:
        flashcard_text.config(
            text=f"{story.correct_emotion}",
            background=emotion_colors[story.correct_emotion],
            font=("Nunito", 28)
        )
        flashcard_image.place_forget()
        flashcard_frame.config(bg=emotion_colors[story.correct_emotion])

        flashcard_text.place(relx=0.5, rely=0.4, anchor="center")
        flashcard_text.after(500, lambda: text_audio(story.correct_emotion))  # Speak after 1 sec
        flashcard_text.after(2000, lambda: robot.executeEmotion(story.correct_emotion.lower()))  # Execute a
    else:
        flashcard_text.config(
            text=story.text,
            background="white",
            font=("Nunito", 28)
        )
        flashcard_image.place(relx=0.5, rely=0.6, anchor="center")
        flashcard_frame.config(bg="white")

        flashcard_text.place(relx=0.5, rely=0.2, anchor="center")

    is_front[0] = not is_front[0]


def navigate(window, new_index):
    if 0 <= new_index < len(stories):
        show_story(window, new_index)

def show_story(window, story_index):
        for widget in window.winfo_children():
            widget.destroy()

        story = stories[story_index]

        base_path = "/home/ros/catkin_ws/src/emotion_based_turtlebot/scripts"
    
        image_path = os.path.join(base_path, story.image_path)

        try:
            img = Image.open(image_path)  
            img = img.resize((500, 400))
            photo = ImageTk.PhotoImage(img)
        except Exception as e:
            print(f"Error loading image: {e}")
            img = Image.new('RGB', (500, 350), color='white')
            photo = ImageTk.PhotoImage(img)
        
        is_front = [True]
        flashcard_frame = tk.Frame(window, bg="white", bd=0, relief="solid")
        flashcard_frame.pack(fill="both", expand=True)

        flashcard_text = tk.Label(
            flashcard_frame,
            text=story.text,
            font=("Nunito", 28),
            wraplength=1100,
            justify="center",
            background="white"
        )
        flashcard_text.place(relx=0.5, rely=0.15, anchor="center")

        flashcard_image = tk.Label(flashcard_frame, image=photo, background="white")
        flashcard_image.photo = photo
        flashcard_image.place(relx=0.5, rely=0.60, anchor="center")

        flashcard_text.bind("<Button-1>", lambda e: flip_flashcard(flashcard_text, flashcard_image, flashcard_frame, story, is_front))
        flashcard_image.bind("<Button-1>", lambda e: flip_flashcard(flashcard_text, flashcard_image, flashcard_frame, story, is_front))
        flashcard_frame.bind("<Button-1>", lambda e: flip_flashcard(flashcard_text, flashcard_image, flashcard_frame, story, is_front))

        separator = tk.Canvas(window, height=2, bg="#d3d3d3", bd=0, relief="flat")
        separator.pack(fill="x")

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
        prev_button.grid(row=0, column=0, padx=60, pady=10)

        next_button = tk.Button(
            nav_frame, text="▶",
            command=lambda: navigate(window, story_index + 1),
            state="normal" if story_index < len(stories) - 1 else "disabled",
            font=("Nunito", 24),
            width=5,
            background="white",
            borderwidth=0
        )
        next_button.grid(row=0, column=1, padx=60, pady=10)
        text_audio(story.text) 





# Score tracking
correct_ans = 0
wrong_ans = 0

def show_instructions(window):
    for widget in window.winfo_children():
        widget.destroy()

    instruction_text = """Welcome to Emotional Learning!

        Goal: 
        - Look at the story and image and try to guess the emotion.
        - Tap the card to flip it and see the correct emotion.

        How to Play: 
        - Click "Next Story" to move to the next one.
        - Flip the card to see if you guessed the right emotion!

        Press "Start" to begin. Have fun!

    """

    instruction_label = tk.Label(window, text=instruction_text, font=("Nunito", 20), wraplength=1000, justify="center")
    instruction_label.pack(pady=50)

    start_button = tk.Button(window, text="Start", command=lambda: start_game(window), font=("Nunito", 20))
    start_button.pack(pady=20)

def start_game(window):
    # Clear the instructions after "Start" is pressed
    for widget in window.winfo_children():
        widget.destroy()

    # Show the first story (after shuffling)
    shuffled_stories = shuffle_stories(stories)
    show_story(window, 0)  # Start with the first story





def run_storytelling(stories):
    root = tk.Tk()
    root.geometry("1200x800")
    root.title("Emotion Learning")
    root.configure(bg="white")

    # Show instructions first
    show_instructions(root)

    # Shuffle the stories for randomness
    shuffled_stories = shuffle_stories(stories)

    # Run the main loop
    root.mainloop()







# Emotion-button click function
def emotion_button(selected_emotion, correct_emotion, score_label):
    global correct_ans, wrong_ans
    if selected_emotion == correct_emotion:
        correct_ans += 1
        tts.speak(f"Yes, I feel {selected_emotion}")
        selected_emotion = selected_emotion if selected_emotion in emotion_colors else "Neutral"
        robot.executeEmotion(selected_emotion.lower())  # Make the robot respond
    else:
        wrong_ans += 1
        tts.speak("That is not the correct emotion. Try again.")

    total = correct_ans + wrong_ans
    score_label.config(text=f"Correct: {correct_ans}  Wrong: {wrong_ans}  Score: {correct_ans}/{total}")


# Reset global variables
correct_ans = 0
wrong_ans = 0

# Run the storytelling application
run_storytelling(stories)
