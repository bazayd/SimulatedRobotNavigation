# Copyright 1996-2025 Cyberbotics Ltd.
# Licensed under the Apache License, Version 2.0

"""
Pioneer 3-DX controller with:
 - Autonomous obstacle avoidance
 - Manual driving via keyboard (M to toggle, arrows/WASD, Space = stop)
 - Voice control via SpeechRecognition (non-blocking background listener)

Voice commands (examples):
  - "manual mode" / "switch to manual" / "take control"
  - "autonomous mode" / "auto mode" / "resume auto"
  - "forward" / "go ahead" / "ahead"
  - "back" / "reverse"
  - "left" / "turn left"
  - "right" / "turn right"
  - "stop" / "halt" / "pause"
"""

from controller import Robot, Keyboard
import threading
import time
import re

# ---- Voice (SpeechRecognition) ----
# pip install SpeechRecognition
try:
    import speech_recognition as sr  # type: ignore
    SR_AVAILABLE = True
except Exception:
    SR_AVAILABLE = False

# === config ===
USE_VOICE = True              # set False to disable voice entirely
VOICE_PHRASE_TIME_LIMIT = 3   # seconds per chunk (short commands work best)
CALIBRATE_SECONDS = 1.0       # ambient noise calibration time on startup
GOOGLE_RECOGNIZER = True      # use online Google recognizer (requires internet)
# To use offline pocketsphinx instead, set GOOGLE_RECOGNIZER=False
# and ensure `pip install pocketsphinx` is installed, then use recognize_sphinx.

# --- key constants (Webots Python has no Keyboard.SPACE) ---
KEY_SPACE = ord(' ')

# --- robot constants ---
MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
DELAY = 70
MAX_SENSOR_VALUE = 1024.0
MIN_DISTANCE = 1.0
WHEEL_WEIGHT_THRESHOLD = 100.0


class State:
    FORWARD = 0
    LEFT = 1
    RIGHT = 2


SENSOR_WEIGHTS = [
    [150.0,   0.0], [200.0,   0.0], [300.0,   0.0], [600.0,   0.0],
    [  0.0, 600.0], [  0.0, 300.0], [  0.0, 200.0], [  0.0, 150.0],
    [  0.0,   0.0], [  0.0,   0.0], [  0.0,   0.0], [  0.0,   0.0],
    [  0.0,   0.0], [  0.0,   0.0], [  0.0,   0.0], [  0.0,   0.0],
]


# -------- Voice control helper --------
class VoiceController:
    """
    Starts a background microphone listener that updates:
      - self.last_cmd: 'fwd' | 'back' | 'left' | 'right' | 'stop' | None
      - self.manual_mode_toggle: True/False (set explicit mode)
    Non-blocking: uses SpeechRecognition.listen_in_background.
    """
    def __init__(self):
        self.available = SR_AVAILABLE and USE_VOICE
        self.recognizer = None
        self.microphone = None
        self.stopper = None  # function returned by listen_in_background
        self.lock = threading.Lock()

        self.last_cmd = None            # driving command
        self.manual_mode = None         # True/False to set manual/auto explicitly
        self.last_text = ""             # last transcript for debugging
        self._running = False

    def start(self):
        if not self.available:
            print("[Voice] SpeechRecognition not available; skipping voice control.")
            return
        try:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            with self.microphone as source:
                print("[Voice] Calibrating microphone noise floor...")
                self.recognizer.adjust_for_ambient_noise(source, duration=CALIBRATE_SECONDS)
                print("[Voice] Calibration done. Listening...")
            # Start background listener
            self.stopper = self.recognizer.listen_in_background(
                self.microphone, self._callback, phrase_time_limit=VOICE_PHRASE_TIME_LIMIT
            )
            self._running = True
        except Exception as e:
            print(f"[Voice] Failed to start listener: {e}")
            self.available = False

    def stop(self):
        if self.stopper:
            self.stopper(wait_for_stop=False)
        self._running = False

    def _callback(self, recognizer, audio):
        try:
            if GOOGLE_RECOGNIZER:
                text = recognizer.recognize_google(audio)
            else:
                # Offline option (requires `pocketsphinx`)
                text = recognizer.recognize_sphinx(audio)
        except sr.UnknownValueError:
            return
        except sr.RequestError as e:
            # Network/API failure
            print(f"[Voice] Recognizer request error: {e}")
            return
        except Exception as e:
            print(f"[Voice] Recognizer error: {e}")
            return

        if not text:
            return
        text = text.lower().strip()
        with self.lock:
            self.last_text = text
        self._interpret(text)

    def _interpret(self, text: str):
        """
        Map natural phrases to robot intents.
        """
        # explicit mode commands
        if re.search(r"\b(manual( mode)?|take control|my control)\b", text):
            with self.lock:
                self.manual_mode = True
            print("[Voice] → Manual mode")
            return
        if re.search(r"\b(auto(nomous)?( mode)?|resume auto|robot control)\b", text):
            with self.lock:
                self.manual_mode = False
            print("[Voice] → Autonomous mode")
            return

        # driving commands
        if re.search(r"\b(stop|halt|pause|freeze)\b", text):
            cmd = "stop"
        elif re.search(r"\b(forward|ahead|go( (ahead|forward))?)\b", text):
            cmd = "fwd"
        elif re.search(r"\b(back|backward|reverse)\b", text):
            cmd = "back"
        elif re.search(r"\b(left|turn left)\b", text):
            cmd = "left"
        elif re.search(r"\b(right|turn right)\b", text):
            cmd = "right"
        else:
            # unrecognized command; ignore silently
            return

        with self.lock:
            self.last_cmd = cmd
        print(f"[Voice] → Command: {cmd}")

    # Safe getters for main loop
    def consume_cmd(self):
        with self.lock:
            cmd = self.last_cmd
            self.last_cmd = None
            return cmd

    def consume_mode(self):
        with self.lock:
            mode = self.manual_mode
            self.manual_mode = None
            return mode


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # devices
    left_wheel = robot.getDevice("left wheel")
    right_wheel = robot.getDevice("right wheel")
    red_led = [
        robot.getDevice("red led 1"),
        robot.getDevice("red led 2"),
        robot.getDevice("red led 3"),
    ]

    sensors = []
    for i in range(MAX_SENSOR_NUMBER):
        ds = robot.getDevice(f"so{i}")
        ds.enable(time_step)
        sensors.append(ds)

    # wheels setup
    left_wheel.setPosition(float("inf"))
    right_wheel.setPosition(float("inf"))
    left_wheel.setVelocity(0.0)
    right_wheel.setVelocity(0.0)

    # keyboard
    kb = robot.getKeyboard()
    kb.enable(time_step)

    # voice
    voice = VoiceController()
    voice.start()

    # state
    led_number = 0
    delay = 0
    state = State.FORWARD
    manual_mode = False
    last_cmd = "stop"  # 'stop' | 'fwd' | 'back' | 'left' | 'right'

    print("Manual mode: OFF (press 'M' to toggle).")
    print("Drive keys (latched): Arrows or WASD. Space = STOP.")
    if voice.available:
        print("Voice control active. Try: 'manual mode', 'forward', 'left', 'stop', 'autonomous mode'.")

    while robot.step(time_step) != -1:
        # -------- keyboard handling --------
        key = kb.getKey()
        while key != -1:
            if key in (ord('M'), ord('m')):
                manual_mode = not manual_mode
                print(f"Manual mode: {'ON' if manual_mode else 'OFF'}")
            elif key == KEY_SPACE:
                last_cmd = "stop"
                left_wheel.setVelocity(0.0)
                right_wheel.setVelocity(0.0)
                print("STOP")
            elif key in (Keyboard.UP, ord('W'), ord('w')):
                last_cmd = "fwd"
            elif key in (Keyboard.DOWN, ord('S'), ord('s')):
                last_cmd = "back"
            elif key in (Keyboard.LEFT, ord('A'), ord('a')):
                last_cmd = "left"
            elif key in (Keyboard.RIGHT, ord('D'), ord('d')):
                last_cmd = "right"
            key = kb.getKey()

        # -------- voice handling (non-blocking) --------
        if voice.available:
            mode = voice.consume_mode()
            if mode is True:
                manual_mode = True
                print("Manual mode: ON (voice)")
            elif mode is False:
                manual_mode = False
                print("Manual mode: OFF (voice)")

            vcmd = voice.consume_cmd()
            if vcmd is not None:
                last_cmd = vcmd

        # -------- LED blink --------
        delay += 1
        if delay == DELAY:
            red_led[led_number].set(0)
            led_number = (led_number + 1) % 3
            red_led[led_number].set(1)
            delay = 0

        # -------- drive logic --------
        if manual_mode:
            # latched tank-drive from last_cmd (set by keyboard or voice)
            if last_cmd == "fwd":
                base, diff = MAX_SPEED, 0.0
            elif last_cmd == "back":
                base, diff = -MAX_SPEED, 0.0
            elif last_cmd == "left":
                base, diff = 0.0, 0.7 * MAX_SPEED
            elif last_cmd == "right":
                base, diff = 0.0, -0.7 * MAX_SPEED
            else:  # stop
                base, diff = 0.0, 0.0
            left_wheel.setVelocity(base + diff)
            right_wheel.setVelocity(base - diff)

        else:
            # autonomous obstacle avoidance
            speed = [0.0, 0.0]
            wheel_weight_total = [0.0, 0.0]

            for i in range(MAX_SENSOR_NUMBER):
                sensor_value = sensors[i].getValue()
                if sensor_value == 0.0:
                    speed_modifier = 0.0
                else:
                    distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE))
                    speed_modifier = 1.0 - (distance / MIN_DISTANCE) if distance < MIN_DISTANCE else 0.0
                wheel_weight_total[0] += SENSOR_WEIGHTS[i][0] * speed_modifier
                wheel_weight_total[1] += SENSOR_WEIGHTS[i][1] * speed_modifier

            if state == State.FORWARD:
                if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
                    speed = [0.7 * MAX_SPEED, -0.7 * MAX_SPEED]
                    state = State.LEFT
                elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
                    speed = [-0.7 * MAX_SPEED, 0.7 * MAX_SPEED]
                    state = State.RIGHT
                else:
                    speed = [MAX_SPEED, MAX_SPEED]
            elif state == State.LEFT:
                if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or
                        wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD):
                    speed = [0.7 * MAX_SPEED, -0.7 * MAX_SPEED]
                else:
                    speed = [MAX_SPEED, MAX_SPEED]
                    state = State.FORWARD
            else:  # RIGHT
                if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or
                        wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD):
                    speed = [-0.7 * MAX_SPEED, 0.7 * MAX_SPEED]
                else:
                    speed = [MAX_SPEED, MAX_SPEED]
                    state = State.FORWARD

            left_wheel.setVelocity(speed[0])
            right_wheel.setVelocity(speed[1])

    # on exit
    if voice.available:
        voice.stop()


if __name__ == "__main__":
    main()
