from controller import Robot
import math
import heapq

# =============================================================================
# NODES - 4 CORNERS
# =============================================================================
NODES = {
    "corner_se": (1.0, -3.0),
    "corner_ne": (1.0, 6.0),
    "corner_nw": (-8.0, 6.0),
    "corner_sw": (-8.0, -3.0)
}

# Graph edges - which corners connect to which
EDGES = {
    "corner_se": ["corner_ne", "corner_sw"],
    "corner_ne": ["corner_se", "corner_nw"],
    "corner_nw": ["corner_ne", "corner_sw"],
    "corner_sw": ["corner_nw", "corner_se"]
}

# =============================================================================
# PATHFINDING - A*
# =============================================================================
def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def astar(start_node, goal_node):
    """Find shortest path from start to goal using A*"""
    frontier = [(0, start_node)]
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal_node:
            break
        
        for neighbor in EDGES[current]:
            new_cost = cost_so_far[current] + euclidean_distance(NODES[current], NODES[neighbor])
            
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + euclidean_distance(NODES[neighbor], NODES[goal_node])
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current
    
    # Reconstruct path
    if goal_node not in came_from:
        return None  # No path found
    
    path = []
    current = goal_node
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    
    return path

# =============================================================================
# ROBOT SETUP
# =============================================================================
robot = Robot()
timestep = 64

gps = robot.getDevice("gps")
gps.enable(timestep)

left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Enable distance sensors
distance_sensors = []
for i in range(16):
    sensor_name = f'so{i}'
    sensor = robot.getDevice(sensor_name)
    if sensor:
        sensor.enable(timestep)
        distance_sensors.append((i, sensor))

# Enable lidar if available
lidar = robot.getDevice("lidar")
if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()
    print("Enabled Lidar")
else:
    lidar = None

for _ in range(10):
    robot.step(timestep)

# =============================================================================
# SENSOR FUNCTIONS
# =============================================================================
def check_front_obstacle():
    """Check if there's an obstacle in front using distance sensors"""
    if len(distance_sensors) == 0:
        return False
    
    front_sensor_indices = [3, 4]
    
    for idx, sensor in distance_sensors:
        if idx in front_sensor_indices:
            value = sensor.getValue()
            if 0.05 < value < 0.5:
                print(f"    ⚠ Front obstacle detected! Sensor so{idx}: {value:.2f}m")
                return True
    
    return False

def check_lidar_obstacle():
    """Check if lidar detects obstacle ahead"""
    if lidar is None:
        return False
    
    ranges = lidar.getRangeImage()
    if not ranges:
        return False
    
    num_points = len(ranges)
    center_start = int(num_points * 0.4)
    center_end = int(num_points * 0.6)
    
    for i in range(center_start, center_end):
        if 0.05 < ranges[i] < 0.5:
            print(f"    ⚠ Lidar detected obstacle at {ranges[i]:.2f}m")
            return True
    
    return False

def check_any_obstacle():
    """Check all sensors for obstacles"""
    return check_front_obstacle() or check_lidar_obstacle()

# =============================================================================
# NAVIGATION FUNCTIONS
# =============================================================================
def get_position():
    vals = gps.getValues()
    return (vals[0], vals[1])

def distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def find_nearest_node():
    """Find which corner we're closest to"""
    current_pos = get_position()
    min_dist = float('inf')
    nearest = None
    
    for name, pos in NODES.items():
        dist = distance(current_pos, pos)
        if dist < min_dist:
            min_dist = dist
            nearest = name
    
    return nearest, min_dist
    
    
def get_wall_distances():
    """Get left and right wall distances from lidar"""
    if lidar is None:
        return None, None
    
    ranges = lidar.getRangeImage()
    if not ranges:
        return None, None
    
    num_points = len(ranges)
    
    # Left side: roughly 90 degrees to the left (quarter of the scan)
    left_start = int(num_points * 0.75)
    left_end = int(num_points * 0.85)
    
    # Right side: roughly 90 degrees to the right (quarter of the scan)
    right_start = int(num_points * 0.15)
    right_end = int(num_points * 0.25)
    
    # Get minimum distance on each side
    left_distances = [ranges[i] for i in range(left_start, left_end) if 0.05 < ranges[i] < 5.0]
    right_distances = [ranges[i] for i in range(right_start, right_end) if 0.05 < ranges[i] < 5.0]
    
    left_min = min(left_distances) if left_distances else None
    right_min = min(right_distances) if right_distances else None
    
    return left_min, right_min    
    

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    for _ in range(5):
        robot.step(timestep)

def check_heading_to_target(target_pos):
    """Check if we're moving directly towards the target"""
    pos1 = get_position()
    
    dist_to_target = distance(pos1, target_pos)
    
    # If very close to target, check for obstacles but assume heading is ok
    if dist_to_target < 0.8:
        print(f"    Very close to target ({dist_to_target:.2f}m)")
        # Still check if path is clear
        if check_any_obstacle():
            print(f"    Obstacle in front while close to target!")
            return False, 999  # Force re-orientation
        return True, 0
    
    dx_desired = target_pos[0] - pos1[0]
    dy_desired = target_pos[1] - pos1[1]
    desired_heading = math.atan2(dy_desired, dx_desired)
    
    for _ in range(30):
        if check_any_obstacle():
            print(f"    Obstacle detected during heading check!")
            stop()
            return False, 999
        
        left_motor.setVelocity(2.0)
        right_motor.setVelocity(2.0)
        robot.step(timestep)
    
    pos2 = get_position()
    stop()
    
    dx_actual = pos2[0] - pos1[0]
    dy_actual = pos2[1] - pos1[1]
    
    if math.sqrt(dx_actual**2 + dy_actual**2) < 0.05:
        print(f"    Didn't move enough to determine heading")
        return False, 999
    
    actual_heading = math.atan2(dy_actual, dx_actual)
    heading_error = normalize_angle(desired_heading - actual_heading)
    
    print(f"    Desired: {math.degrees(desired_heading):.1f}°, Actual: {math.degrees(actual_heading):.1f}°, Error: {math.degrees(heading_error):.1f}°")
    
    is_aligned = abs(heading_error) < math.radians(15)
    return is_aligned, heading_error

def turn_until_facing_target(target_pos):
    """Keep turning until heading directly towards the target"""
    print(f"  Orienting towards target {target_pos}...")
    
    max_attempts = 25
    attempt = 0
    
    while attempt < max_attempts:
        attempt += 1
        
        is_aligned, heading_error = check_heading_to_target(target_pos)
        
        if is_aligned:
            print(f"    ✓ Aligned with target!")
            return True
        
        print(f"    Attempt {attempt}: Correcting {math.degrees(heading_error):.1f}° error...")
        
        turn_steps = int(abs(heading_error) / (math.pi/2) * 25)
        turn_steps = max(10, min(turn_steps, 60))
        
        if heading_error > 0:
            for _ in range(turn_steps):
                left_motor.setVelocity(-1.5)
                right_motor.setVelocity(1.5)
                robot.step(timestep)
        else:
            for _ in range(turn_steps):
                left_motor.setVelocity(1.5)
                right_motor.setVelocity(-1.5)
                robot.step(timestep)
        
        stop()
    
    print(f"    Warning: Could not align perfectly")
    return False

def drive_to_node(target_pos, name):
    """Drive to a target node with obstacle detection and wall following"""
    print(f"  Driving to {name} at {target_pos}")
    
    step = 0
    max_steps = 2000
    last_check = 0
    
    while robot.step(timestep) != -1 and step < max_steps:
        step += 1
        
        current_pos = get_position()
        dist = distance(current_pos, target_pos)
        
        if step % 100 == 0:
            print(f"    Step {step}: Distance: {dist:.2f}m")
        
        if dist < 0.5:
            stop()
            print(f"  ✓ Reached {name}!")
            return True
        
        # Check for obstacles
        if check_any_obstacle():
            print(f"    ⚠ OBSTACLE DETECTED! Recalibrating...")
            stop()
            
            # Back up
            print(f"    Backing up...")
            for _ in range(20):
                left_motor.setVelocity(-1.5)
                right_motor.setVelocity(-1.5)
                robot.step(timestep)
            stop()
            
            # Re-orient towards target
            print(f"    Re-orienting...")
            turn_until_facing_target(target_pos)
            
            last_check = step
            continue
        
        # Periodic heading check if far from target
        if dist > 1.5 and step - last_check > 150:
            last_check = step
            stop()
            
            is_aligned, heading_error = check_heading_to_target(target_pos)
            
            if not is_aligned:
                print(f"    Drifted off course! Re-orienting...")
                turn_until_facing_target(target_pos)
        
        # Base speed
        base_speed = min(2.5, dist * 1.5)
        
        # Wall-following correction using lidar
        left_wall, right_wall = get_wall_distances()
        
        correction = 0
        # Only apply wall-following if BOTH walls are detected AND reasonably close
        # This means we're in a hallway, not at a corner
        if (left_wall is not None and right_wall is not None and 
            left_wall < 2.0 and right_wall < 2.0 and
            abs(left_wall - right_wall) < 1.5):  # Walls roughly balanced
            
            # Try to stay centered between walls
            wall_diff = right_wall - left_wall
            correction = wall_diff * 0.3
            
            # Limit correction
            correction = max(-0.5, min(0.5, correction))
            
            if step % 100 == 0:
                print(f"    Hallway mode: L={left_wall:.2f}m, R={right_wall:.2f}m, correction={correction:.2f}")
        else:
            # At a corner or open space - no wall following
            if step % 100 == 0:
                print(f"    Open/corner mode: L={left_wall}, R={right_wall}")
        
        # Apply differential steering
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        # Clamp speeds
        left_speed = max(0, min(3.0, left_speed))
        right_speed = max(0, min(3.0, right_speed))
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
    stop()
    return False

# =============================================================================
# MAIN - NAVIGATE TO CHOSEN DESTINATION
# =============================================================================
print("="*60)
print("GRAPH-BASED NAVIGATION TO DESTINATION")
print("="*60)

# ========== SET YOUR DESTINATION HERE ==========
DESTINATION = "corner_sw"  # OPTIONS: corner_se, corner_ne, corner_nw, corner_sw
# ================================================

print(f"\nDESTINATION: {DESTINATION}")

# Find current position
start_pos = get_position()
print(f"Current position: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")

# Find nearest node
nearest_node, dist_to_nearest = find_nearest_node()
print(f"Nearest node: {nearest_node} (distance: {dist_to_nearest:.2f}m)")

# If far from any node, go to nearest first
if dist_to_nearest > 1.0:
    print(f"\nGoing to {nearest_node} first...")
    target_pos = NODES[nearest_node]
    turn_until_facing_target(target_pos)
    drive_to_node(target_pos, nearest_node)
    # Update nearest node
    nearest_node = nearest_node

# Find path using A*
path = astar(nearest_node, DESTINATION)

if path is None:
    print(f"\nERROR: No path found from {nearest_node} to {DESTINATION}!")
else:
    print(f"\nPlanned path: {' -> '.join(path)}")
    print("="*60)
    
    # Navigate along the path
    for i in range(len(path)-1):
        current_node = path[i]
        next_node = path[i+1]
        
        next_pos = NODES[next_node]
        
        print(f"\n[{i+1}/{len(path)-1}] {current_node} -> {next_node}")
        
        # Turn and drive to next node
        turn_until_facing_target(next_pos)
        success = drive_to_node(next_pos, next_node)
        
        if not success:
            print("Navigation failed!")
            break
        
        for _ in range(30):
            robot.step(timestep)
    
    print("\n" + "="*60)
    print(f"REACHED DESTINATION: {DESTINATION}")
    print("="*60)

stop()