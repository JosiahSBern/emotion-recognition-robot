import tkinter as tk
import random
from collections import defaultdict
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import Image, ImageTk
import json
import pandas as pd

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

def load_stories_from_json(filename="stories/filtered_stories.json"):
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


class Teaching():
    def __init__(self, window, stories):
        super().__init__(window)
        self.stories = stories
        self.shuffled_stories = self.shuffle_stories(stories)
        self.emotion_colors = emotion_colors  # assumes global dictionary
        self.show_story(0)

    def shuffle_stories(self, stories):
        random.shuffle(stories)
        return stories

    def robot_execute(self, emotion):
        pass  # optional: send emotion to robot, e.g., self.robot.act(emotion)

    def flip_flashcard(self, flashcard_text, flashcard_image, flashcard_frame, story, is_front, caption_label):
        if is_front[0]:
            flashcard_text.config(
                text=story.correct_emotion,
                background=self.emotion_colors[story.correct_emotion],
                font=("Nunito", 28)
            )
            flashcard_image.place_forget()
            flashcard_frame.config(bg=self.emotion_colors[story.correct_emotion])
            flashcard_text.place(relx=0.5, rely=0.4, anchor="center")
            caption_label.config(text="This shows the correct emotion for the story.")
            self.robot_execute(story.correct_emotion)
        else:
            flashcard_text.config(
                text=story.text,
                background="white",
                font=("Nunito", 28)
            )
            flashcard_image.place(relx=0.5, rely=0.6, anchor="center")
            flashcard_frame.config(bg="white")
            flashcard_text.place(relx=0.5, rely=0.2, anchor="center")
            caption_label.config(text="Click to reveal the correct emotion.")
        is_front[0] = not is_front[0]

    def show_story(self, story_index):
        self.clear()

        story = self.shuffled_stories[story_index]

        img = Image.open(story.image_path).resize((500, 400))
        photo = ImageTk.PhotoImage(img)

        is_front = [True]

        flashcard_frame = tk.Frame(self.window, bg="white")
        flashcard_frame.pack(fill="both", expand=True)

        caption_label = tk.Label(
            self.window,
            text="Click on the story to reveal its correct emotion.",
            font=("Nunito", 18),
            background="white"
        )
        caption_label.pack(pady=10)

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
        flashcard_image.image = photo  # prevent garbage collection
        flashcard_image.place(relx=0.5, rely=0.60, anchor="center")

        # bind click to all interactive elements
        for widget in [flashcard_text, flashcard_image, flashcard_frame, caption_label]:
            widget.bind("<Button-1>", lambda e: self.flip_flashcard(
                flashcard_text, flashcard_image, flashcard_frame, story, is_front, caption_label))

        # separator
        separator = tk.Canvas(self.window, height=2, bg="#d3d3d3", bd=0, relief="flat")
        separator.pack(fill="x")

        # navigation
        nav_frame = tk.Frame(self.window, background="white")
        nav_frame.pack(pady=20)

        tk.Button(
            nav_frame, text="◀",
            command=lambda: self.show_story(story_index - 1),
            state="normal" if story_index > 0 else "disabled",
            font=("Nunito", 24), width=5,
            background="white", borderwidth=0
        ).grid(row=0, column=0, padx=60)
        tk.Label(nav_frame, text="Go back", font=("Nunito", 14), background="white").grid(row=1, column=0, pady=2)

        tk.Button(
            nav_frame, text="▶",
            command=lambda: self.show_story(story_index + 1),
            state="normal" if story_index < len(self.shuffled_stories) - 1 else "disabled",
            font=("Nunito", 24), width=5,
            background="white", borderwidth=0
        ).grid(row=0, column=1, padx=60)
        tk.Label(nav_frame, text="Next", font=("Nunito", 14), background="white").grid(row=1, column=1, pady=2)

    def run_storytelling(self):
        self.window.geometry("1400x900")
        self.window.title("Emotion Learning")
        self.window.configure(bg="white")
        self.window.mainloop()


import tkinter as tk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Global variables for scoring and stats
correct_ans = 0
wrong_ans = 0
selected_emotions = []
correct_stats = {}
incorrect_stats = {}

# Emotion App
class EmotionApp:
    def __init__(self, stories):
        self.window = tk.Tk()
        self.window.geometry("1400x900")
        self.window.title("Emotion Storytelling Game")
        self.window.configure(bg="white")

        self.stories = stories
        self.show_start()

    def show_start(self):
        StartScreen(self.window, self.stories, self.show_teaching, self.show_learning)

    def show_teaching(self, stories):
        Teaching(self.window, self, stories, emotion_colors)

    def show_learning(self, stories, story_index=0):
        StoryPage(self.window, self, stories, story_index)

    def run(self):
        self.window.mainloop()

# Start Screen
class StartScreen:
    def __init__(self, window, stories, show_teaching, show_learning):
        self.window = window
        self.stories = stories
        self.show_teaching = show_teaching
        self.show_learning = show_learning
        self.draw()

    def draw(self):
        self.clear()

        title = tk.Label(self.window, text="Welcome to the Emotion App", font=("Nunito", 32), bg="white")
        title.pack(pady=60)
    
        instructions = (
            "Instructions:\n\n"
            "Read each story carefully.\n"
            "Look at the picture.\n"
            "Choose the emotion that best matches the story.\n"
            "Get a robot dance for every correct answer.\n"
            "Try to get the highest score!"
        )

        instructions_label = tk.Label(
            self.window,
            text=instructions,
            font=("Nunito", 20),
            justify="center",
            padx=30,
            bg="white",
            wraplength=1000
        )
        instructions_label.pack(pady=20)

        button_frame = tk.Frame(self.window, bg="white")
        button_frame.pack(pady=60) 

        teach_btn = tk.Button(
            button_frame,
            text="Teaching Mode",
            font=("Nunito", 20),
            bg="#e0c158",
            command=lambda: self.show_teaching(self.stories)
        )
        teach_btn.grid(row=0, column=0, padx=60)

        quiz_btn = tk.Button(
            button_frame,
            text="Quiz Mode",
            font=("Nunito", 20),
            bg="#6da7d1",
            command=lambda: self.show_learning(self.stories)
        )
        quiz_btn.grid(row=0, column=1, padx=60)

    def clear(self):
        for widget in self.window.winfo_children():
            widget.destroy()

# Teaching Page (inherits from tk.Frame)
class Teaching:
    def __init__(self, window, app, stories, emotion_colors):
        self.window = window
        self.app = app
        self.stories = stories
        self.emotion_colors = emotion_colors
        self.shuffled_stories = self.shuffle_stories(stories)
        self.show_story(0)

    def shuffle_stories(self, stories):
        random.shuffle(stories)
        return stories

    def text_audio(self, text):
        print(text)

    def robot_execute(self, emotion):
        pass

    def flip_flashcard(self, flashcard_text, flashcard_image, flashcard_frame, story, is_front, caption_label):
        if is_front[0]:
            flashcard_text.config(
                text=f"{story.correct_emotion}",
                background=self.emotion_colors[story.correct_emotion],
                font=("Nunito", 28)
            )
            flashcard_image.place_forget()
            flashcard_frame.config(bg=self.emotion_colors[story.correct_emotion])
            flashcard_text.place(relx=0.5, rely=0.4, anchor="center")
            caption_label.config(text="This shows the correct emotion for the story.")
            self.robot_execute(flashcard_text)
        else:
            flashcard_text.config(
                text=story.text,
                background="white",
                font=("Nunito", 28)
            )
            flashcard_image.place(relx=0.5, rely=0.6, anchor="center")
            flashcard_frame.config(bg="white")
            flashcard_text.place(relx=0.5, rely=0.2, anchor="center")
            caption_label.config(text="Click to reveal the correct emotion.")

        is_front[0] = not is_front[0]

    def show_story(self, story_index):
        story = self.stories[story_index]

        for widget in self.window.winfo_children():
            widget.destroy()

        # Back button bar frame (top left, always visible)
        top_bar = tk.Frame(self.window, bg="white")
        top_bar.place(x=0, y=0, relwidth=1)

        self.back_button = tk.Button(
            top_bar,
            text="←",
            font=("Nunito", 16),
            bg="white",
            relief="flat",
            command=self.app.show_start,
            cursor="hand2"
        )
        self.back_button.pack(side="left", padx=10, pady=10)

        image_path = story.image_path
        img = Image.open(image_path)
        img = img.resize((500, 400))
        photo = ImageTk.PhotoImage(img)

        is_front = [True]

        flashcard_frame = tk.Frame(self.window, bg="white", bd=0, relief="solid")
        flashcard_frame.pack(fill="both", expand=True, pady=(40, 0))  # Push down to not cover back button

        caption_label = tk.Label(self.window, text="Click on the story to reveal its correct emotion.", font=("Nunito", 18), background="white")
        caption_label.pack(pady=10)

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

        flip_callback = lambda e: self.flip_flashcard(flashcard_text, flashcard_image, flashcard_frame, story, is_front, caption_label)
        flashcard_text.bind("<Button-1>", flip_callback)
        flashcard_image.bind("<Button-1>", flip_callback)
        flashcard_frame.bind("<Button-1>", flip_callback)
        caption_label.bind("<Button-1>", flip_callback)

        separator = tk.Canvas(self.window, height=2, bg="#d3d3d3", bd=0, relief="flat")
        separator.pack(fill="x")

        nav_frame = tk.Frame(self.window, background="white")
        nav_frame.pack(pady=20)

        prev_button = tk.Button(
            nav_frame, text="◀",
            command=lambda: self.navigate(story_index - 1),
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
            command=lambda: self.navigate(story_index + 1),
            state="normal" if story_index < len(self.stories) - 1 else "disabled",
            font=("Nunito", 24),
            width=5,
            background="white",
            borderwidth=0
        )
        next_button.grid(row=0, column=1, padx=60)

        next_caption = tk.Label(nav_frame, text="Next", font=("Nunito", 14), background="white")
        next_caption.grid(row=1, column=1, padx=60, pady=2)

    def navigate(self, new_index):
        if 0 <= new_index < len(self.stories):
            self.show_story(new_index)
        else:
            self.app.show_start()

# Score Frame
class ScoreFrame:
    def __init__(self, parent):
        global score_label
        frame = tk.Frame(parent, bg="white")
        frame.pack(side="top", anchor="ne", padx=40, pady=20)
        score_label = tk.Label(frame, text=f"{correct_ans} / {correct_ans + wrong_ans}", font=("Nunito", 28, "bold"), bg="white")
        score_label.pack()

# Base Page
class BasePage:
    def __init__(self, window):
        self.window = window
        self.clear()

    def clear(self):
        for widget in self.window.winfo_children():
            widget.destroy()

class StoryPage(BasePage):
    def __init__(self, window, app, stories, story_index):
        super().__init__(window)
        self.app = app
        global score_label
        ScoreFrame(window)

        # Assuming stories is a list of Story objects
        story = stories[story_index]

        # Story Text
        story_label = tk.Label(window, text=story.text, font=("Nunito", 24), wraplength=1000, justify="center", bg="white")
        story_label.pack()

        # Story Image
        image_frame = tk.Frame(window, bg="white")
        image_frame.pack(anchor="center")
        img = Image.open(story.image_path)  # Access image path
        img = img.resize((500, 350))
        photo = ImageTk.PhotoImage(img)
        image_label = tk.Label(image_frame, image=photo)
        image_label.image = photo
        image_label.pack()

        # Emotion Buttons
        emotion_frame = tk.Frame(window, bg="white")
        emotion_frame.pack(anchor="center", pady=20)

        for i, emotion in enumerate(story.emotions):  # Access emotions list
            tk.Button(
                emotion_frame,
                text=emotion,
                bg=emotion_colors.get(emotion, "#ffffff"),  # Get color based on emotion
                command=lambda e=emotion: self.handle_emotion(e, story.correct_emotion, story_index),
                width=30,
                height=5,
                font=("Nunito", 14)
            ).grid(row=0, column=i, padx=20)

        # Navigation Buttons
        nav_frame = tk.Frame(window, bg="white")
        nav_frame.pack(pady=20)

        tk.Button(
            nav_frame, text="\u25C0",
            command=lambda: navigate(window, story_index - 1),
            state="normal" if story_index > 0 else "disabled",
            font=("Nunito", 24), width=5,
            bg="white", borderwidth=0
        ).grid(row=0, column=0, padx=60)

        tk.Label(nav_frame, text="Go back", font=("Nunito", 14), bg="white").grid(row=1, column=0, padx=60)

        tk.Button(
            nav_frame, text="\u25B6",
            command=lambda: navigate(window, story_index + 1) if story_index < len(stories) - 1 else EndPage(window),
            font=("Nunito", 24), width=5,
            bg="white", borderwidth=0
        ).grid(row=0, column=1, padx=60)

        tk.Label(nav_frame, text="Next", font=("Nunito", 14), bg="white").grid(row=1, column=1, padx=60)

        # Add a top-left back button
        back_button = tk.Button(
            self.window,
            text="←",
            font=("Nunito", 16),
            bg="white",
            relief="flat",
            command=self.app.show_start,
            cursor="hand2"
        )
        back_button.place(x=20, y=20)
        
    def handle_emotion(self, selected_emotion, correct_emotion, story_index):
        global correct_ans, wrong_ans

        selected_emotions.append(selected_emotion)

        if selected_emotion == correct_emotion:
            correct_ans += 1
            result_text = "CORRECT! ✅"
            result_color = "green"
            # robot_execute("happy")
            correct_stats[correct_emotion] = correct_stats.get(correct_emotion, 0) + 1
        else:
            wrong_ans += 1
            result_text = "WRONG! ❌"
            result_color = "red"
            incorrect_stats[correct_emotion] = incorrect_stats.get(correct_emotion, 0) + 1

        score_label.config(text=f"{correct_ans} / {correct_ans + wrong_ans}")

        self.clear()

        feedback_frame = tk.Frame(self.window, background="white")
        feedback_frame.pack(expand=True, fill="both")

        tk.Label(
            feedback_frame,
            text=result_text,
            fg=result_color,
            font=("Nunito", 35, "bold"),
            padx=10,
            bg="white"
        ).pack(expand=True)

        if result_text == "CORRECT! ✅":
            tk.Label(
                feedback_frame,
                text="ROBOT DANCE!",
                fg=result_color,
                font=("Nunito", 35, "bold"),
                padx=10,
                bg="white"
            ).pack(expand=True)

        button_frame = tk.Frame(feedback_frame, background="white")
        button_frame.pack(anchor="n", pady=(40, 60), padx=30)

        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)

        tk.Button(
            button_frame,
            text="Try Again?",
            command=lambda: navigate(self.window, story_index),
            font=("Nunito", 28),
            borderwidth=2,
            relief="solid",
            background="white"
        ).grid(row=0, column=0, sticky="ew", padx=60, pady=150)

        tk.Button(
            button_frame,
            text="Go Next",
            command=lambda: navigate(self.window, story_index + 1) if story_index < len(stories) - 1 else EndPage(self.window),
            font=("Nunito", 28),
            borderwidth=2,
            relief="solid",
            background="white"
        ).grid(row=0, column=1, sticky="ew", padx=60, pady=150)
        
# End Page
class EndPage(BasePage):
    def __init__(self, window):
        super().__init__(window)

        global correct_ans, wrong_ans

        summary_frame = tk.Frame(window, pady=40, bg="white")
        summary_frame.pack()

        tk.Label(
            summary_frame,
            text=f"Final Score: {correct_ans} / {correct_ans + wrong_ans}",
            font=("Nunito", 36, "bold"), fg="black", bg="white"
        ).pack(pady=20)

        tk.Label(
            summary_frame, text="Emotion Stats", font=("Nunito", 28, "bold"), pady=10, bg="white"
        ).pack()

        stats_box = tk.Frame(summary_frame, bg="white")
        stats_box.pack()

        emotions = sorted(set(correct_stats.keys()) | set(incorrect_stats.keys()))
        correct_counts = [correct_stats.get(e, 0) for e in emotions]
        incorrect_counts = [incorrect_stats.get(e, 0) for e in emotions]

        max_correct = max(correct_counts) if correct_counts else 0
        max_incorrect = max(incorrect_counts) if incorrect_counts else 0
        y_max = max(max_correct, max_incorrect) + 1

        figure = plt.Figure(figsize=(16, 10), dpi=100)
        canvas = FigureCanvasTkAgg(figure, window)
        canvas.get_tk_widget().pack(fill='both', expand=True)

        ax1 = figure.add_subplot(1, 2, 1)
        ax1.bar(emotions, correct_counts, color='green')
        ax1.set_title('Correct Emotions')
        ax1.set_xlabel('Emotion')
        ax1.set_ylabel('Count')
        ax1.set_ylim(0, y_max)

        ax2 = figure.add_subplot(1, 2, 2)
        ax2.bar(emotions, incorrect_counts, color='red')
        ax2.set_title('Incorrect Emotions')
        ax2.set_xlabel('Emotion')
        ax2.set_ylabel('Count')
        ax2.set_ylim(0, y_max)

        # Add a top-left back button
        back_button = tk.Button(
            self.window,
            text="←",
            font=("Nunito", 16),
            bg="white",
            relief="flat",
            command=self.app.show_start,
            cursor="hand2"
        )
        back_button.place(x=20, y=20)
# Navigation Function
def navigate(window, story_index):
    if 0 <= story_index < len(stories):
        StoryPage(window, app, stories, story_index)
    else:
        EndPage(window)


# Run the app
if __name__ == "__main__":
    stories = load_stories_from_json()
    app = EmotionApp(stories)
    app.run()