"""pioneer_collision_avoidance_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import numpy as np
import cv2
import pytesseract
from controller import Robot, Camera, Display

#check ultralytics version

MAX_SPEED = 8.25

MAX_SENSOR_NUMBER = 16

MAX_SENSOR_VALUE = 1024

MIN_DISTANCE = 1.0

WHEEL_WEIGHT_THRESHOLD = 100

DELAY = 70


#  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  # {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}},
  # {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},
  # {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}},   {.wheel_weight = {0, 0}}};

# For use of rear sensors, they are arranged as such:
# so8 to so11 are for the rear-left arc
# so12 to so15 are rear right arc
# weights weaker than front ones, rear sensors act as nudges or guards.
SENSOR_WEIGHTS = [
   (150, 0), (300,0), (800, 0), (1000,0),
   (0, 1000), (0, 800), (0,300), (0, 150),
   (0,0), (0,0), (0,0), (0,0),
   (0,0), (0,0), (0,0), (0,0)
]

# Similar to C enum range of25 3 states
FORWARD, LEFT, RIGHT = range(3)


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
time_step = int(robot.getBasicTimeStep())

# initiliaze left and right wheel device names
# set position
left_wheel = robot.getDevice("left wheel")
right_wheel = robot.getDevice("right wheel")
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))

# Define and initialize leds for debugging
red_leds = [robot.getDevice("red led 1"),
            robot.getDevice("red led 2"),
            robot.getDevice("red led 3")]


# Sensors list
# for i in range of 16 max sensors
# ds is sensor name (so0, so1, so2), equivalent to C's buffer string method sprintf
# enable time step and append ds to sensors list
sensors = []
for i in range(MAX_SENSOR_NUMBER):
    name = f"so{i}"
    try:
        ds = robot.getDevice(name)
        if ds is None:
            print(f"Device {name} not found!")
        else:
            ds.enable(time_step)
            sensors.append(ds)
    except Exception as e:
        print(f"Error getting {name}: {e}")


# set current state, led number, and delay value
state = FORWARD
led_number = 0
delay = 0


camera = robot.getDevice("camera")
camera.enable(time_step)
height = camera.getHeight()
width = camera.getWidth()

# YOLOv8n model

# loop for program time step
while robot.step(time_step) != -1:
# current wheel weight for left and right wheels
# speed set to 0
    wheel_weight_total = [0.0, 0.0]
    speed = [0.0, 0.0]
    
    # Allows us to get camera view in simulation
    image = camera.getImage()
 
    # image array and img_rgb
    img_array = np.frombuffer(image, np.uint8).reshape((height,width), 4)
    img_rgb = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
    
    #using pytesseract to get text from camera image view
    text = pytesseract.image_to_string(image)
    print(text)

    width = camera.getWidth()
    height = camera.getHeight()

    # Sensor processing
    # values of ds, through all 16 sensors
    for i, ds in enumerate(sensors):
    # get the value of ds (current sensor value for the sensor at index i)
    # if value is 0 then speed modifier is 0
        value = ds.getValue()
        print(f"Current sensor value: {value}")
        if value == 0.0:
            modifier = 0.0
        # otherwise, if obstacle is detected, use distance to calculate and see if within min distance.
        else:
            distance = 5.0 * (1.0 - (value / MAX_SENSOR_VALUE))
            print(f"Current distance: {distance}")
            if distance < MIN_DISTANCE:
                print(f"Distance {distance} within min distance")
                modifier = 1 - (distance / MIN_DISTANCE)
            else:
                modifier = 0.0


        # w set to sensor weight at index i
        # cumulative wheel weight of left wheel is added to current sensor weight times the speed modifiere
        # cumulative wheel weight of right wheel is added to current sensor weight times the speed modifier
        w = SENSOR_WEIGHTS[i]
        wheel_weight_total[0] += w[0] * modifier
        wheel_weight_total[1] += w[1] * modifier
    


    # State machine
    if state == FORWARD:
        # if wheel weight greater than 100 threshold for wheel weight, set left wheel speed greater (slowly) and right wheel backward
        #set state to left
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
            print(f"Wheel weight (left): {wheel_weight_total[0]} greater than threshold")
            speed[0] = 0.7*MAX_SPEED
            speed[1] = -0.7*MAX_SPEED
            state = LEFT
         # do the opposite as before is right wheel greater than 100 threshold 
         # set state to right 
        elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            print(f"Wheel weight (right): {wheel_weight_total[1]} greater than threshold")
            speed[0] = -0.7*MAX_SPEED
            speed[1] = 0.7*MAX_SPEED
            state = RIGHT
        else:
            speed = [MAX_SPEED, MAX_SPEED]

    elif state == LEFT:
    # if state is set to left check if left wheel weight greater than threshold or vice versa for right wheel
    # if so, speed increased for left and right wheel slowler or backwards.
    # Else, keep going forward 
    # This is to avoid left wheel not detecting obstacles while turning 
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed[0] = 0.7*MAX_SPEED
            speed[1] = -0.7*MAX_SPEED
        else:
            speed = [MAX_SPEED, MAX_SPEED]
            state = FORWARD
    
    # Same for right state (as wheels are turning right)
    elif state == RIGHT:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed[0] = -0.7*MAX_SPEED
            speed[1] = 0.7*MAX_SPEED
        else:
            speed = [MAX_SPEED, MAX_SPEED]
            state = FORWARD
    print(f"Speed after obstacle: (left wheel: {speed[0]}), (right wheel: {speed[1]})")
  
    # LED cycle for debugging
    delay += 1
    if delay == DELAY:
        red_leds[led_number].set(0)
        led_number = (led_number + 1) % 3
        red_leds[led_number].set(1)
        delay = 0

    # Apply speeds
    left_wheel.setVelocity(speed[0])
    right_wheel.setVelocity(speed[1])