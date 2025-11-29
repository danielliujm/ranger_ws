#!/usr/bin/env python3
import os
import sys
import time
import json
import re
import pyttsx3
import speech_recognition as sr
import google.generativeai as genai
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.node import Node
import rclpy

import ctypes


class AlpacaInteractionNode(Node):
    def __init__(self):
        super().__init__('alpaca_interaction_node')
        # Initialization code here

        self.coord_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
    
    def publish_coords (self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.coord_pub.publish(msg)


try:
    # Redirect ALSA errors to /dev/null
    def py_alsa_silent(*args):
        pass

    # Set error handler
    ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                          ctypes.c_char_p, ctypes.c_int,
                                          ctypes.c_char_p)

    c_error_handler = ERROR_HANDLER_FUNC(py_alsa_silent)
    asound = ctypes.cdll.LoadLibrary("libasound.so")
    asound.snd_lib_error_set_handler(c_error_handler)
except Exception as e:
    print("Could not silence ALSA:", e)


def state(msg):
    print(f"\n>>> {msg}\n")
    sys.stdout.flush()


def say(text):
    """Text-to-speech"""

    engine = pyttsx3.init(driverName = 'espeak')
    # voices = engine.getProperty('voices')
    # engine.setProperty('voice',voices[11].id) #English
    # engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()


def state(msg):
    print(f"\n>>> {msg}\n")
    sys.stdout.flush()


# -----------------------------------------------------
# CONFIG
# -----------------------------------------------------
GENAI_API_KEY = "AIzaSyB4akC2LeTmRn7551CoVEQNXaegcKx9ypw"
genai.configure(api_key=GENAI_API_KEY)

WAKE_PHRASES = ["hey alpaca", "alpaca", "hello alpaca", "can you help me", "help"]
LANGUAGE = "en-US"
COMMAND_TIMEOUT = 25

# -----------------------------------------------------
# Room coordinates
# -----------------------------------------------------
ROOM_COORDS = {
    3140: [0.3, 17.706],
    3141: [0.3, 17.706],
    3150: [0.3, 8.743],
    3160: [3.2, 7.1],
    3161: [4.567, -4.880],
    3170: [0.3, -9.543],
    3171: [0.3, -9.543],
}


def get_coords(room_number):
    return ROOM_COORDS.get(room_number, None)


# -----------------------------------------------------
# LLM extraction
# -----------------------------------------------------
EXTRACTION_PROMPT = """
You are a navigation interpreter. ONLY output JSON.

Task:
- Identify the target room number.
- Convert number-words to digits.
- Ignore irrelevant text.

Output EXACTLY this format:
{{
  "target_room": <int or null>,
}}

User said: "{utterance}"
"""


def extract_room(utterance):
    prompt = EXTRACTION_PROMPT.format(utterance=utterance)
    model = genai.GenerativeModel("gemini-2.5-flash")

    try:
        response = model.generate_content(prompt)
        raw = response.text.replace("```json", "").replace("```", "").strip()

        match = re.search(r"\{[\s\S]*\}", raw)
        if not match:
            return None

        data = json.loads(match.group(0))
        return data.get("target_room")

    except Exception as e:
        print("[LLM ERROR]", e)
        return None


# -----------------------------------------------------
# Speech helpers
# -----------------------------------------------------
def passive_listen(r, mic):
    """Wait for wake phrase."""
    state("Passive Listening — say 'Hey Alpaca'...")
    with mic as source:
        r.adjust_for_ambient_noise(source, duration=0.3)
        audio = r.listen(source, phrase_time_limit=4)

    try:
        text = r.recognize_google(audio, language=LANGUAGE).lower()
        print(f"[hearing] {text}")
        return any(w in text for w in WAKE_PHRASES)
    except:
        return False


def capture_sentence(r, mic):
    state("[Listening for navigation request...]")
    with mic as source:
        audio = r.listen(source, timeout=COMMAND_TIMEOUT, phrase_time_limit=COMMAND_TIMEOUT)

    state("[processing…]")
    try:
        text = r.recognize_google(audio, language=LANGUAGE)
        state(f"Captured: {text}")
        return text.strip()
    except:
        state("Could not understand.")
        return ""


# -----------------------------------------------------
# MAIN LOOP
# -----------------------------------------------------
def guidebot_main():
    r = sr.Recognizer()
    mic = sr.Microphone()

    rclpy.init()

    node = AlpacaInteractionNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("Alpaca Interaction Node started.")

    state("GuideBot started.")

    while True:
        # ---- Passive listening mode ----
        if not passive_listen(r, mic):
            continue

        # Wake word detected
        say("How can I help you?")
        utterance = capture_sentence(r, mic)

        if not utterance:
            say("Sorry, I didn't catch that.")
            continue

        # Extract target room
        target_room = extract_room(utterance)
        if not target_room:
            say("I could not find a room number in your request.")
            continue

        coords = get_coords(target_room)
        
        if coords is None:
            say(f"Sorry, I don't know where room {target_room} is.")
            print(f"Unknown room {target_room}")
            continue

        node.publish_coords(coords[0], coords[1])  # Publish target coordinates

        # Confirm to user
        say(f"Okay, guiding to room {target_room}.")
        print("\n=== Navigation Command ===")
        print(f"Destination Room: {target_room}")
        print(f"Coordinates: {coords}")
        print("==========================\n")

        # Return immediately to passive listening
        state("Returning to passive listening...")


# -----------------------------------------------------
# Text input helpers (non-speech path)
# -----------------------------------------------------
def process_utterance(utterance, node):
    """Process a plain-text utterance through the existing pipeline.

    This mirrors the behavior used after speech recognition so callers
    can supply text directly for testing or unreliable STT.
    """
    utterance = (utterance or "").strip()
    if not utterance:
        say("Sorry, I didn't catch that.")
        return False

    # Extract target room
    target_room = extract_room(utterance)
    if not target_room:
        say("I could not find a room number in your request.")
        return False

    coords = get_coords(target_room)
    if coords is None:
        say(f"Sorry, I don't know where room {target_room} is.")
        print(f"Unknown room {target_room}")
        return False

    # Publish target coordinates
    node.publish_coords(coords[0], coords[1])

    # Confirm to user
    say(f"Okay, guiding to room {target_room}.")
    print("\n=== Navigation Command ===")
    print(f"Destination Room: {target_room}")
    print(f"Coordinates: {coords}")
    print("==========================\n")

    return True


def text_input_mode(node):
    """Interactive loop accepting plain text utterances from the user.

    Callers can create the `AlpacaInteractionNode` then call this function
    to route typed input through the same pipeline used for speech.
    """
    state("Text-input mode — type requests (or 'quit' to exit)")
    try:
        while True:
            try:
                line = input("utterance> ")
            except EOFError:
                break

            if not line:
                continue
            if line.strip().lower() in ("quit", "exit"):
                state("Exiting text-input mode.")
                break

            process_utterance(line, node)
    except KeyboardInterrupt:
        state("Exiting text-input mode.")



def main():
    # Allow selecting text-input mode via command-line flag
    use_text_mode = False
    args = sys.argv[1:]
    if any(a in ("--text", "--text-input", "-t") for a in args):
        use_text_mode = True

    try:
        if use_text_mode:
            rclpy.init()
            node = AlpacaInteractionNode()
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                text_input_mode(node)
            finally:
                rclpy.shutdown()
                print("Shutting down.")
        else:
            try:
                guidebot_main()
            except KeyboardInterrupt:
                rclpy.shutdown()
                print("Shutting down.")
    except KeyboardInterrupt:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("Shutting down.")


if __name__ == "__main__":
    main()

