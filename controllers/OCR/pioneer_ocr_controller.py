

from controller import Robot, Motor, DistanceSensor, LED, Camera
from enum import Enum
import pytesseract
from PIL import Image
import numpy as np

# Constants
MAX_SPEED = 5.24  # maximal speed allowed
MAX_SENSOR_NUMBER = 16  # how many sensors are on the robot
DELAY = 70  # delay used for the blinking LEDs
MAX_SENSOR_VALUE = 1024  # maximal value returned by the sensors
MIN_DISTANCE = 1.0  # minimal distance, in meters, for an obstacle to be considered
WHEEL_WEIGHT_THRESHOLD = 100  # minimal weight for the robot to turn


class State(Enum):
    """Enum to represent the state of the robot"""
    FORWARD = 1
    LEFT = 2
    RIGHT = 3


class SensorData:
    """Class to store the data associated with one sensor"""
    def __init__(self, device_tag=None, wheel_weight=None):
        self.device_tag = device_tag
        self.wheel_weight = wheel_weight if wheel_weight else [0.0, 0.0]


class Pioneer3DXController:
    """Controller class for Pioneer 3-DX robot with OCR capabilities"""
    
    def __init__(self):
        """Initialize the robot and its components"""
        # Initialize Webots robot
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_wheel = self.robot.getDevice("left wheel")
        self.right_wheel = self.robot.getDevice("right wheel")
        self._setup_motors()
        
        # Initialize LEDs
        self.red_leds = [
            self.robot.getDevice("red led 1"),
            self.robot.getDevice("red led 2"),
            self.robot.getDevice("red led 3")
        ]
        
        # Initialize distance sensors with wheel weights
        self.sensors = self._setup_sensors()
        
        # Initialize camera for OCR (optional - may not exist on all models)
        self.camera = None
        self.ocr_enabled = True
        self._setup_camera()
        
        # State variables
        self.state = State.FORWARD
        self.led_number = 0
        self.delay_counter = 0
        
        # OCR configuration
        self.ocr_interval = 10  # Process OCR every N iterations
        self.ocr_counter = 0
        self.last_recognized_text = ""
        
    def _setup_motors(self):
        """Configure the wheel motors"""
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
        
    def _setup_sensors(self):
        """Initialize distance sensors with their wheel weight configurations"""
        # Wheel weight configuration (how much each sensor affects robot direction)
        wheel_weights = [
            [150, 0], [200, 0], [300, 0], [600, 0],  # Front-right sensors
            [0, 600], [0, 300], [0, 200], [0, 150],  # Front-left sensors
            [0, 0], [0, 0], [0, 0], [0, 0],          # Side/rear sensors (unused)
            [0, 0], [0, 0], [0, 0], [0, 0]
        ]
        
        sensors = []
        for i in range(MAX_SENSOR_NUMBER):
            sensor_name = f"so{i}"
            device = self.robot.getDevice(sensor_name)
            device.enable(self.time_step)
            sensors.append(SensorData(device, wheel_weights[i]))
            
        return sensors
    
    def _setup_camera(self):
        """Initialize camera for OCR if available"""
        try:
            self.camera = self.robot.getDevice("camera")
            if self.camera:
                self.camera.enable(self.time_step)
                self.ocr_enabled = True
                print("Camera initialized for OCR")
        except:
            print("No camera found - OCR disabled")
            self.ocr_enabled = False
    
    def process_ocr(self):
    
        import numpy as np
        """Process camera image with Tesseract OCR"""
        if not self.ocr_enabled or not self.camera:
            return None
        
        try:
            # Get image from camera
            image_data = self.camera.getImage()
            if not image_data:
                return None
            
            # Convert to numpy array
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            
            # Webots images are in BGRA format
            image_array = np.frombuffer(image_data, dtype=np.uint8)
            image_array = image_array.reshape((height, width, 4))
            
            # Convert BGRA to RGB for PIL
            image_rgb = image_array[:, :, [2, 1, 0]]  # Swap B and R channels
            
            # Create PIL Image
            pil_image = Image.fromarray(image_rgb)
            
            #using grayscale (converting to grayscale)
            gray = pil_image.convert('L')
            
            from PIL import ImageEnhance
            enhancer = ImageEnhance.Contrast(gray)
            gray = enhancer.enhance(2.0)
            
            from PIL import ImageFilter
            gray = gray.filter(ImageFilter.SHARPEN)
            
            new_width = width * 2
            new_height = height * 2
            gray = gray.resize((new_width, new_height), Image.Resampling.LANCZOS)
            
            import numpy as np
            gray_array = np.array(gray)
            
            threshold = 128
            binary = (gray_array > threshold) * 255
            binary_image = Image.fromarray(binary.astype(np.uint8))
            
                 # Try multiple PSM modes for better results
            configs = [
                '--psm 6 --oem 3',  # Uniform block of text
                '--psm 7 --oem 3',  # Single text line
                '--psm 11 --oem 3', # Sparse text
                '--psm 3 --oem 3',  # Fully automatic
            ]
            
            best_text = ""
            max_confidence = 0
            
            for config in configs:
                try:
                    # Get detailed data
                    data = pytesseract.image_to_data(binary_image, config=config, output_type=pytesseract.Output.DICT)
                    
                    # Filter by confidence
                    text_parts = []
                    for i, conf in enumerate(data['conf']):
                        if int(conf) > 60:  # Only include high-confidence results
                            text = data['text'][i].strip()
                            if text:
                                text_parts.append(text)
                    
                    detected_text = ' '.join(text_parts)
                    avg_conf = sum([int(c) for c in data['conf'] if int(c) > 0]) / max(len([c for c in data['conf'] if int(c) > 0]), 1)
                    
                    if avg_conf > max_confidence:
                        max_confidence = avg_conf
                        best_text = detected_text
                        
                except:
                    continue
            
            # Use best result
            text = best_text
            
            # Only print if meaningful text detected
            if text and len(text) > 2 and text != self.last_recognized_text:
                print(f"OCR Detected: '{text}' (confidence: {max_confidence:.1f}%)")
                self.last_recognized_text = text
                
                # Optional: Save debug images
                if hasattr(self, 'save_debug_images') and self.save_debug_images:
                    import os
                    os.makedirs("ocr_debug", exist_ok=True)
                    pil_image.save(f"ocr_debug/original_{len(text)}.png")
                    binary_image.save(f"ocr_debug/processed_{len(text)}.png")
                
            return text
            
        except Exception as e:
            print(f"OCR Error: {e}")
            import traceback
            traceback.print_exc()
            return None
   
    
    
    def calculate_obstacle_avoidance(self):
        """Calculate wheel speeds based on sensor readings"""
        wheel_weight_total = [0.0, 0.0]
        
        for sensor in self.sensors:
            sensor_value = sensor.device_tag.getValue()
            
            # If sensor doesn't detect anything, skip it
            if sensor_value == 0.0:
                speed_modifier = 0.0
            else:
                # Compute actual distance to obstacle
                distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE))
                
                # Calculate influence based on distance
                if distance < MIN_DISTANCE:
                    speed_modifier = 1.0 - (distance / MIN_DISTANCE)
                else:
                    speed_modifier = 0.0
            
            # Add modifier for both wheels
            wheel_weight_total[0] += sensor.wheel_weight[0] * speed_modifier
            wheel_weight_total[1] += sensor.wheel_weight[1] * speed_modifier
        
        return wheel_weight_total
    
    def update_state(self, wheel_weight_total):
        """Update robot state based on sensor weights"""
        speed = [0.0, 0.0]
        
        if self.state == State.FORWARD:
            if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
                # Turn left
                speed[0] = 0.7 * MAX_SPEED
                speed[1] = -0.7 * MAX_SPEED
                self.state = State.LEFT
            elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
                # Turn right
                speed[0] = -0.7 * MAX_SPEED
                speed[1] = 0.7 * MAX_SPEED
                self.state = State.RIGHT
            else:
                # Go forward
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                
        elif self.state == State.LEFT:
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or 
                wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD):
                # Continue turning left
                speed[0] = 0.7 * MAX_SPEED
                speed[1] = -0.7 * MAX_SPEED
            else:
                # Return to forward
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                self.state = State.FORWARD
                
        elif self.state == State.RIGHT:
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or 
                wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD):
                # Continue turning right
                speed[0] = -0.7 * MAX_SPEED
                speed[1] = 0.7 * MAX_SPEED
            else:
                # Return to forward
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                self.state = State.FORWARD
        
        return speed
    
    def update_leds(self):
        """Update LED blinking pattern"""
        self.delay_counter += 1
        if self.delay_counter >= DELAY:
            self.red_leds[self.led_number].set(0)
            self.led_number = (self.led_number + 1) % 3
            self.red_leds[self.led_number].set(1)
            self.delay_counter = 0
    
    def run(self):
        """Main control loop"""
        print("Pioneer 3-DX Controller Started")
        print(f"OCR {'Enabled' if self.ocr_enabled else 'Disabled'}")
        
        while self.robot.step(self.time_step) != -1:
            # Calculate obstacle avoidance
            wheel_weight_total = self.calculate_obstacle_avoidance()
            
            # Update robot state and get speeds
            speed = self.update_state(wheel_weight_total)
            
            # Set motor speeds
            self.left_wheel.setVelocity(speed[0])
            self.right_wheel.setVelocity(speed[1])
            
            # Update LEDs
            self.update_leds()
            
            # Process OCR periodically
            if self.ocr_enabled:
                self.ocr_counter += 1
                if self.ocr_counter >= self.ocr_interval:
                    self.process_ocr()
                    self.ocr_counter = 0


def main():
    """Main entry point"""
    controller = Pioneer3DXController()
    controller.run()


if __name__ == "__main__":
    main()