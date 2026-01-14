from controller import Robot
from collections import deque
import math

# -----------------------------
# CONFIGURATION
# -----------------------------
timestep = 32
forward_speed = 1.0        # moderate forward speed
rotation_speed = 1.0       # speed for turning in place
pos_tolerance = 0.3        # distance to consider node reached
angle_threshold = 0.02     # radians to consider heading aligned
clearance_distance = 0.4   # move this far after turning to clear node

# -----------------------------
# NODES & EDGES (corners example)
# -----------------------------
NODES = {
    "C1": (1.0, -3.0),
    "C2": (1.0, 6.0),
    "C3": (-8.0, 6.0),
    "C4": (-8.0, -3.0)
}

EDGES = {
    "C1": ["C2", "C4"],
    "C2": ["C1", "C3"],
    "C3": ["C2", "C4"],
    "C4": ["C1", "C3"]
}

# -----------------------------
# BFS PATHFINDING
# -----------------------------
def bfs_path(start, goal):
    visited = set()
    parent = {}
    queue = deque([start])
    visited.add(start)
    while queue:
        node = queue.popleft()
        if node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return list(reversed(path))
        for neighbor in EDGES[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = node
                queue.append(neighbor)
    return None

# -----------------------------
# INITIALIZE ROBOT
# -----------------------------
robot = Robot()
gps = robot.getDevice("gps")
compass = robot.getDevice("compass1")
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

gps.enable(timestep)
compass.enable(timestep)

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# -----------------------------
# HELPER FUNCTIONS
# -----------------------------
def get_pose():
    x, y = gps.getValues()[:2]
    c = compass.getValues()
    theta = math.atan2(c[0], c[2])
    return x, y, theta

def angle_diff(a, b):
    d = a - b
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

def distance(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

# -----------------------------
# TURN IN PLACE
# -----------------------------
def turn_to(target_pos):
    while robot.step(timestep) != -1:
        x, y, theta = get_pose()
        dx = target_pos[0] - x
        dy = target_pos[1] - y
        target_theta = math.atan2(dy, dx)
        err = angle_diff(target_theta, theta)
        if abs(err) < angle_threshold:
            break
        # rotate in place
        left_motor.setVelocity(-rotation_speed if err > 0 else rotation_speed)
        right_motor.setVelocity(rotation_speed if err > 0 else -rotation_speed)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# -----------------------------
# DRIVE FORWARD CLEARANCE
# -----------------------------
def move_forward_clearance(distance_to_move):
    moved = 0.0
    left_motor.setVelocity(forward_speed)
    right_motor.setVelocity(forward_speed)
    while robot.step(timestep) != -1 and moved < distance_to_move:
        # approximate distance moved per timestep
        moved += forward_speed * (timestep / 1000.0)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# -----------------------------
# DRIVE TO NODE
# -----------------------------
def drive_to(target_pos):
    reached = False
    while robot.step(timestep) != -1 and not reached:
        x, y, _ = get_pose()
        if distance((x, y), target_pos) < pos_tolerance:
            reached = True
            break
        left_motor.setVelocity(forward_speed)
        right_motor.setVelocity(forward_speed)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# -----------------------------
# START / GOAL
# -----------------------------
start_node = "C1"
goal_node = "C3"

PATH = bfs_path(start_node, goal_node)
if PATH is None:
    print("No path found!")
    exit()
print("BFS Path:", PATH)

# -----------------------------
# INITIAL SENSOR STEP
# -----------------------------
# stabilize GPS and compass
for _ in range(10):
    robot.step(timestep)

# -----------------------------
# FOLLOW PATH
# -----------------------------
for node_name in PATH[1:]:
    target_pos = NODES[node_name]
    # 1. turn first
    turn_to(target_pos)
    # 2. move forward a bit to clear the node
    move_forward_clearance(clearance_distance)
    # 3. then drive normally to node
    drive_to(target_pos)

print("Reached destination", goal_node)
