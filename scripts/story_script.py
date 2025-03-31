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





class Story:
    def __init__(self, text, emotions, correct_emotion,image_path):
        self.text = text
        self.emotions = emotions
        self.correct_emotion = correct_emotion
        self.image_path = image_path

# Initialize ROS and robot components
rospy.init_node('storytelling_node', anonymous=True)
robot = Turtlebot()
tts = TextToSpeech()
MAX_QUESTIONS = 5

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

# Load and shuffle stories
stories = load_stories_from_json()
random.shuffle(stories)
stories = stories[:MAX_QUESTIONS]


correct_ans = 0
wrong_ans = 0
selected_emotions = []

def text_audio(text):
    tts.speak(text)
    print(text)


text_audio("Test")

def emotion_button(selected_emotion, correct_emotion, score_label, root, story_index):
    global correct_ans, wrong_ans
    if selected_emotion == correct_emotion:
        correct_ans += 1
        text_audio(f"Yes, I feel {selected_emotion}")
        robot.executeEmotion(selected_emotion.lower())
    else:
        wrong_ans += 1
        text_audio("That is not the correct emotion. Try again.")

    total = correct_ans + wrong_ans
    score_label.config(text=f"Score:{correct_ans} / {total}")

def show_results(window):
    for widget in window.winfo_children():
        widget.destroy()

    final_score_label = tk.Label(window, text=f"Final Score: {correct_ans} / {correct_ans + wrong_ans}",
                                 font=("Nunito", 24))
    final_score_label.pack(pady=20)

    plot_results(window)

    restart_button = tk.Button(window, text="Restart Quiz", command=lambda: restart_game(window), font=("Nunito", 20))
    restart_button.pack(pady=20)

def plot_results(window):
    correct_answers = defaultdict(int)
    wrong_answers = defaultdict(int)

    for i, emotion in enumerate(selected_emotions):
        if i < len(stories):
            story = stories[i]
            if emotion == story.correct_emotion:
                correct_answers[emotion] += 1
            else:
                wrong_answers[emotion] += 1

    emotions_correct = list(correct_answers.keys())
    counts_correct = [correct_answers[emotion] for emotion in emotions_correct]
    colors_correct = [emotion_colors.get(emotion, "#ffffff") for emotion in emotions_correct]

    emotions_wrong = list(wrong_answers.keys())
    counts_wrong = [wrong_answers[emotion] for emotion in emotions_wrong]
    colors_wrong = [emotion_colors.get(emotion, "#ffffff") for emotion in emotions_wrong]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    ax1.bar(emotions_correct, counts_correct, color=colors_correct)
    ax1.set_title("Correct Answers per Emotion")
    
    ax2.bar(emotions_wrong, counts_wrong, color=colors_wrong)
    ax2.set_title("Wrong Answers per Emotion")

    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().pack(pady=30)

def restart_game(window):
    global correct_ans, wrong_ans, selected_emotions

    correct_ans = 0
    wrong_ans = 0
    selected_emotions = []

    show_story(window, 0)



def show_story(window, story_index):
    global score_label, current_story_index
    current_story_index = story_index  

    # Clear the window
    for widget in window.winfo_children():
        widget.destroy()

    # Get the current story and display the image
    story = stories[story_index]
    base_path = "/home/ros/catkin_ws/src/emotion_based_turtlebot/scripts"
    image_path = os.path.join(base_path, story.image_path)

    try:
        img = Image.open(image_path)  
        img = img.resize((500, 400))  # Resize to a fixed size
        photo = ImageTk.PhotoImage(img)
    except Exception as e:
        print(f"Error loading image: {e}")
        img = Image.new('RGB', (500, 350), color='white')
        photo = ImageTk.PhotoImage(img)
    
    # Display the image in a grid
    image_frame = tk.Frame(window)
    image_frame.grid(row=0, column=0, columnspan=4, padx=20, pady=20)  # Take up the full width
    image_label = tk.Label(image_frame, image=photo)
    image_label.image = photo
    image_label.grid(row=0, column=0, padx=20, pady=20)

    # Display the score label
    score_label = tk.Label(window, text=f"Score:{correct_ans} / {correct_ans + wrong_ans}", font=("Nunito", 20))
    score_label.grid(row=0, column=3, sticky="ne", padx=20, pady=20)  # Place in the top-right corner

    # Dynamically scale font size based on window size
    window_width = window.winfo_width()
    font_size = max(16, int(window_width / 50))  # Adjust the formula to control text scaling

    # Display the story text in a grid
    story_label = tk.Label(window, text=story.text, font=("Nunito", font_size), wraplength=window_width * 0.8, justify="center")
    story_label.grid(row=1, column=0, columnspan=4, padx=20, pady=20)

    # Read the story out loud
    text_audio(story.text)

    # Create emotion buttons frame and place them in the grid
    emotion_frame = tk.Frame(window)
    emotion_frame.grid(row=2, column=0, columnspan=4, pady=20)  # Take up the full width

    # Dynamically adjust button layout based on window size
    num_buttons = len(story.emotions)
    
    for i, emotion in enumerate(story.emotions):
        color = emotion_colors.get(emotion, "#ffffff")
        button = tk.Button(
            emotion_frame,
            text=emotion,
            command=lambda emotion=emotion: emotion_button(emotion, story.correct_emotion, score_label, window, story_index),  # Pass root and story_index here
            activebackground=color,
            background=color,
            font=("Nunito", font_size),  # Adjust font size for buttons as well
        )

        # Make the buttons stretch to the width of the window (adjust columns in the grid)
        button.grid(row=0, column=i, padx=20, sticky="ew", ipadx=10, ipady=10)

        # Configure column weights to make them resize properly
        emotion_frame.grid_columnconfigure(i, weight=1, uniform="equal")

    # Ensure the emotion buttons adjust to the size of the window
    window.grid_columnconfigure(0, weight=1)
    window.grid_columnconfigure(1, weight=1)
    window.grid_columnconfigure(2, weight=1)
    window.grid_columnconfigure(3, weight=1)

    # Next button to go to the next story
    next_button = tk.Button(window, text="Next Story", command=lambda: show_story(window, story_index + 1) if story_index + 1 < len(stories) else show_results(window), font=("Nunito", font_size))
    next_button.grid(row=3, column=0, columnspan=4, pady=20)  # Take up the full width



def show_instructions(window):
    for widget in window.winfo_children():
        widget.destroy()

    instruction_text = """Welcome to the Stories Quiz!
    
    - Read the story carefully.
    - Look at the image and think about the emotions it conveys.
    - Choose the correct emotion from the options.
    - Your score is displayed in the top right corner.
    - Click 'Next Story' to continue.

    Press 'Start' when you're ready!
    """

    instruction_label = tk.Label(window, text=instruction_text, font=("Nunito", 20), wraplength=1000, justify="center")
    instruction_label.pack(pady=50)

    start_button = tk.Button(window, text="Start", command=lambda: show_story(window, 0), font=("Nunito", 20))
    start_button.pack(pady=20)

def run_storytelling():
    root = tk.Tk()
    root.geometry("1440x900")
    root.title("Stories Quiz")
    show_instructions(root) 
    root.mainloop()





run_storytelling()