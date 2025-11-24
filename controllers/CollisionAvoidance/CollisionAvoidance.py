
from controller import Robot, Motor, DistanceSensor, LED

# maximal speed allowed
MAX_SPEED = 5.24

# how many sensors are on the robot
MAX_SENSOR_NUMBER = 16

# delay used for the blinking leds
DELAY = 70

# maximal value returned by the sensors
MAX_SENSOR_VALUE = 1024.0

# minimal distance, in meters, for an obstacle to be considered
MIN_DISTANCE = 1.0

# minimal weight for the robot to turn
WHEEL_WEIGHT_THRESHOLD = 100

# how much each sensor affects the direction of the robot (left, right)
SENSOR_WHEEL_WEIGHTS = [
    [150, 0], [200, 0], [300, 0], [600, 0],
    [0, 600], [0, 300], [0, 200], [0, 150],
    [0, 0],   [0, 0],   [0, 0],   [0, 0],
    [0, 0],   [0, 0],   [0, 0],   [0, 0]
]

# robot states
FORWARD = "FORWARD"
LEFT = "LEFT"
RIGHT = "RIGHT"

def main():
    robot = Robot()

    # stores simulation time step
    time_step = int(robot.getBasicTimeStep())

    # wheels
    left_wheel = robot.getDevice("left wheel")
    right_wheel = robot.getDevice("right wheel")

    # LEDs
    red_led = [
        robot.getDevice("red led 1"),
        robot.getDevice("red led 2"),
        robot.getDevice("red led 3")
    ]

    # distance sensors
    sensors = []
    for i in range(MAX_SENSOR_NUMBER):
        name = f"so{i}"
        ds = robot.getDevice(name)
        ds.enable(time_step)
        sensors.append(ds)

    # setup wheels
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(0.0)
    right_wheel.setVelocity(0.0)

    led_number = 0
    delay = 0

    # by default, the robot goes forward
    state = FORWARD

    # main loop
    while robot.step(time_step) != -1:
        # initialize speed and wheel_weight_total arrays at the beginning of the loop
        speed = [0.0, 0.0]
        wheel_weight_total = [0.0, 0.0]

        # read sensors and compute influence on wheel weights
        for i in range(MAX_SENSOR_NUMBER):
            sensor_value = sensors[i].getValue()

            if sensor_value == 0.0:
                speed_modifier = 0.0
            else:
                # computes the actual distance to the obstacle, given the value returned by the sensor
                distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE))  # lookup table inverse.
                if distance < MIN_DISTANCE:
                    speed_modifier = 1.0 - (distance / MIN_DISTANCE)
                else:
                    speed_modifier = 0.0

            wheel_weight_total[0] += SENSOR_WHEEL_WEIGHTS[i][0] * speed_modifier
            wheel_weight_total[1] += SENSOR_WHEEL_WEIGHTS[i][1] * speed_modifier

        # simplistic state machine for direction
        if state == FORWARD:
            if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
                speed[0] = 0.7 * MAX_SPEED
                speed[1] = -0.7 * MAX_SPEED
                state = LEFT
            elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
                speed[0] = -0.7 * MAX_SPEED
                speed[1] = 0.7 * MAX_SPEED
                state = RIGHT
            else:
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED

        elif state == LEFT:
            if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
                speed[0] = 0.7 * MAX_SPEED
                speed[1] = -0.7 * MAX_SPEED
            else:
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                state = FORWARD

        elif state == RIGHT:
            if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
                speed[0] = -0.7 * MAX_SPEED
                speed[1] = 0.7 * MAX_SPEED
            else:
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                state = FORWARD

        # blink LEDs periodically
        delay += 1
        if delay == DELAY:
            # switch off current, move to next, switch it on
            try:
                red_led[led_number].set(0)
            except Exception:
                # some Webots versions use set(0/1) while others might accept set(0)/set(1)
                pass
            led_number = (led_number + 1) % 3
            try:
                red_led[led_number].set(1)
            except Exception:
                pass
            delay = 0

        # set motor speeds
        left_wheel.setVelocity(speed[0])
        right_wheel.setVelocity(speed[1])

    # No explicit cleanup needed in the Python controller

if __name__ == "__main__":
    main()