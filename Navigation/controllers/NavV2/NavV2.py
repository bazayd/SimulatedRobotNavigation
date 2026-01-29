from controller import Robot
import math
import heapq
import json
import os

# =============================================================================
# NODE CLASSES
# =============================================================================
class Node:
    """Base class for all nodes in the navigation graph"""
    def __init__(self, name, position, node_type="waypoint"):
        self.name = name
        self.position = position
        self.node_type = node_type
        self.neighbors = []
    
    def __repr__(self):
        return f"{self.node_type.capitalize()}('{self.name}', {self.position})"

class CornerNode(Node):
    """Corner node - intersection of two hallways"""
    def __init__(self, name, position):
        super().__init__(name, position, node_type="corner")

class DoorNode(Node):
    """Door node - entrance to a room"""
    def __init__(self, name, position, room_number=None):
        super().__init__(name, position, node_type="door")
        self.room_number = room_number
    
    def __repr__(self):
        room_info = f", Room {self.room_number}" if self.room_number else ""
        return f"Door('{self.name}', {self.position}{room_info})"

# =============================================================================
# NAVIGATION GRAPH
# =============================================================================
class NavigationGraph:
    """Graph structure for navigation"""
    def __init__(self):
        self.nodes = {}
    
    def add_node(self, node):
        self.nodes[node.name] = node
    
    def add_edge(self, node1_name, node2_name, bidirectional=True):
        if node1_name in self.nodes and node2_name in self.nodes:
            if node2_name not in self.nodes[node1_name].neighbors:
                self.nodes[node1_name].neighbors.append(node2_name)
            if bidirectional and node1_name not in self.nodes[node2_name].neighbors:
                self.nodes[node2_name].neighbors.append(node1_name)
    
    def get_position(self, node_name):
        return self.nodes[node_name].position if node_name in self.nodes else None
    
    def get_neighbors(self, node_name):
        return self.nodes[node_name].neighbors if node_name in self.nodes else []
    
    def find_door_by_room(self, room_number):
        for name, node in self.nodes.items():
            if isinstance(node, DoorNode) and node.room_number == room_number:
                return name
        return None

# =============================================================================
# LOAD GRAPH FROM JSON
# =============================================================================
def load_graph_from_file(filename):
    graph = NavigationGraph()
    
    controller_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(controller_dir, filename)
    
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except FileNotFoundError:
        print(f"ERROR: Config file not found at {config_path}")
        return None
    
    for node_data in config['nodes']:
        if node_data['type'] == 'corner':
            node = CornerNode(node_data['name'], tuple(node_data['position']))
        elif node_data['type'] == 'door':
            node = DoorNode(
                node_data['name'], 
                tuple(node_data['position']), 
                room_number=node_data.get('room_number')
            )
        else:
            node = Node(node_data['name'], tuple(node_data['position']))
        
        graph.add_node(node)
    
    for edge in config['edges']:
        graph.add_edge(edge[0], edge[1])
    
    return graph

# =============================================================================
# PATHFINDING - A*
# =============================================================================
def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def astar(graph, start_node, goal_node):
    frontier = [(0, start_node)]
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
    
    while frontier:
        _, current = heapq.heappop(frontier)
        
        if current == goal_node:
            break
        
        for neighbor in graph.get_neighbors(current):
            new_cost = cost_so_far[current] + euclidean_distance(
                graph.get_position(current), 
                graph.get_position(neighbor)
            )
            
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + euclidean_distance(
                    graph.get_position(neighbor), 
                    graph.get_position(goal_node)
                )
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current
    
    if goal_node not in came_from:
        return None
    
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

# Enable lidar
lidar = robot.getDevice("lidar")
if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()
else:
    lidar = None

# Enable camera
camera = robot.getDevice("camera")
if camera:
    camera.enable(timestep)
    print("Camera enabled")
else:
    camera = None
    print("WARNING: No camera - room verification disabled")

# Try to import OCR libraries
try:
    import numpy as np
    from PIL import Image
    import pytesseract
    HAS_OCR = True
    print("OCR available")
except ImportError:
    HAS_OCR = False
    print("WARNING: OCR not available")

for _ in range(10):
    robot.step(timestep)

# =============================================================================
# SENSOR FUNCTIONS
# =============================================================================
def check_front_obstacle():
    if len(distance_sensors) == 0:
        return False
    
    front_sensor_indices = [3, 4]
    obstacle_count = 0
    
    for idx, sensor in distance_sensors:
        if idx in front_sensor_indices:
            value = sensor.getValue()
            if 0.1 < value < 0.4:
                obstacle_count += 1
    
    return obstacle_count >= 2

def check_lidar_obstacle():
    if lidar is None:
        return False
    
    ranges = lidar.getRangeImage()
    if not ranges:
        return False
    
    num_points = len(ranges)
    center_start = int(num_points * 0.45)
    center_end = int(num_points * 0.55)
    
    obstacle_count = 0
    for i in range(center_start, center_end):
        if 0.15 < ranges[i] < 0.5:
            obstacle_count += 1
    
    return obstacle_count >= 3

def check_any_obstacle():
    return check_front_obstacle() or check_lidar_obstacle()

def get_wall_distances():
    if lidar is None:
        return None, None
    
    ranges = lidar.getRangeImage()
    if not ranges:
        return None, None
    
    num_points = len(ranges)
    
    left_start = int(num_points * 0.75)
    left_end = int(num_points * 0.85)
    
    right_start = int(num_points * 0.15)
    right_end = int(num_points * 0.25)
    
    left_distances = [ranges[i] for i in range(left_start, left_end) if 0.05 < ranges[i] < 5.0]
    right_distances = [ranges[i] for i in range(right_start, right_end) if 0.05 < ranges[i] < 5.0]
    
    left_min = min(left_distances) if left_distances else None
    right_min = min(right_distances) if right_distances else None
    
    return left_min, right_min

# =============================================================================
# ROOM VERIFICATION
# =============================================================================
def verify_room_number_ocr(expected_room):
    """Verify room number using OCR with image preprocessing"""
    if camera is None:
        print("  No camera - skipping verification")
        return True
    
    if not HAS_OCR:
        print("  No OCR - skipping verification")
        return True
    
    width = camera.getWidth()
    height = camera.getHeight()
    image_data = camera.getImage()
    
    if not image_data:
        print("  No image data")
        return False
    
    try:
        import cv2
        
        # Convert to numpy array
        image_array = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        image_rgb = image_array[:, :, [2, 1, 0]]
        
        # Convert to grayscale
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        
        # Apply thresholding to get black and white image
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # Apply morphological operations to clean up
        kernel = np.ones((2,2), np.uint8)
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        # Invert if text is white on black
        if np.mean(cleaned) > 127:
            cleaned = cv2.bitwise_not(cleaned)
        
        # Save processed image
        debug_dir = os.path.dirname(os.path.abspath(__file__))
        cv2.imwrite(os.path.join(debug_dir, f"processed_{expected_room}.png"), cleaned)
        
        # Convert back to PIL for OCR
        from PIL import Image
        pil_image = Image.fromarray(cleaned)
        
        # Save for viewing
        pil_image.save(os.path.join(debug_dir, f"for_ocr_{expected_room}.png"))
        print(f"  Saved processed image")
        
        # Try different OCR configs
        results = []
        
        # Config 1: Single line
        text1 = pytesseract.image_to_string(pil_image, config='--psm 7 --oem 3')
        nums1 = ''.join(filter(str.isdigit, text1))
        if nums1:
            results.append(nums1)
            print(f"  PSM 7: '{text1.strip()}' -> {nums1}")
        
        # Config 2: Single word  
        text2 = pytesseract.image_to_string(pil_image, config='--psm 8 --oem 3')
        nums2 = ''.join(filter(str.isdigit, text2))
        if nums2:
            results.append(nums2)
            print(f"  PSM 8: '{text2.strip()}' -> {nums2}")
        
        # Config 3: Treat as digits only
        text3 = pytesseract.image_to_string(pil_image, config='--psm 7 digits')
        nums3 = ''.join(filter(str.isdigit, text3))
        if nums3:
            results.append(nums3)
            print(f"  Digits: '{text3.strip()}' -> {nums3}")
        
        if not results:
            print("  ✗ No numbers detected")
            return False
        
        # Find most common result or longest
        from collections import Counter
        counts = Counter(results)
        detected_numbers = counts.most_common(1)[0][0]
        
        print(f"  Best result: {detected_numbers}")
        
        detected_room = int(detected_numbers)
        if detected_room == expected_room:
            print(f"  ✓ Verified: Room {detected_room}")
            return True
        else:
            print(f"  ✗ Wrong room! Expected {expected_room}, saw {detected_room}")
            return False
            
    except Exception as e:
        print(f"  OCR error: {e}")
        import traceback
        traceback.print_exc()
        return False

def verify_room_at_door(door_node):
    """Turn to face door, back up for better view, verify, then return"""
    print(f"  Verifying Room {door_node.room_number}...")
    
    door_pos = door_node.position
    
    # Get current heading
    pos1 = get_position()
    for _ in range(10):
        left_motor.setVelocity(1.0)
        right_motor.setVelocity(1.0)
        robot.step(timestep)
    pos2 = get_position()
    stop()
    
    dx_current = pos2[0] - pos1[0]
    dy_current = pos2[1] - pos1[1]
    
    if math.sqrt(dx_current**2 + dy_current**2) < 0.01:
        current_heading = 0
    else:
        current_heading = math.atan2(dy_current, dx_current)
    
    # Determine which wall door is on and face it
    if abs(door_pos[1] - 6.0) < 0.5:  # North wall
        target_heading = math.pi / 2  # Face north (90°)
    elif abs(door_pos[1] - (-3.0)) < 0.5:  # South wall
        target_heading = -math.pi / 2  # Face south (-90°)
    elif abs(door_pos[0] - 1.0) < 0.5:  # East wall
        target_heading = 0  # Face east (0°)
    elif abs(door_pos[0] - (-8.0)) < 0.5:  # West wall
        target_heading = math.pi  # Face west (180°)
    else:
        target_heading = current_heading
    
    # Calculate turn needed
    angle_to_turn = normalize_angle(target_heading - current_heading)
    
    # Turn to face door
    turn_steps = int(abs(angle_to_turn) / (math.pi/2) * 25)
    turn_steps = max(5, min(turn_steps, 50))
    
    if abs(angle_to_turn) > math.radians(5):
        if angle_to_turn > 0:
            for _ in range(turn_steps):
                left_motor.setVelocity(1.5)
                right_motor.setVelocity(-1.5)
                robot.step(timestep)
        else:
            for _ in range(turn_steps):
                left_motor.setVelocity(-1.5)
                right_motor.setVelocity(1.5)
                robot.step(timestep)
        stop()
    
    # BACK UP for better camera angle
    print("  Backing up for better view...")
    backup_steps = 50  # Adjust this value for more/less backup distance
    
    for _ in range(backup_steps):
        left_motor.setVelocity(-1.5)
        right_motor.setVelocity(-1.5)
        robot.step(timestep)
    
    stop()
    
    # Wait for camera to stabilize
    print("  Taking picture...")
    for _ in range(50):
        robot.step(timestep)
    
    # Verify
    verified = verify_room_number_ocr(door_node.room_number)
    
    # MOVE FORWARD back to door position
    print("  Returning to door...")
    for _ in range(backup_steps):
        left_motor.setVelocity(1.5)
        right_motor.setVelocity(1.5)
        robot.step(timestep)
    
    stop()
    
    # Turn back to hallway orientation
    if abs(angle_to_turn) > math.radians(5):
        if angle_to_turn > 0:
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
    
    return verified
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

def find_nearest_node(graph):
    current_pos = get_position()
    min_dist = float('inf')
    nearest = None
    
    for name, node in graph.nodes.items():
        dist = distance(current_pos, node.position)
        if dist < min_dist:
            min_dist = dist
            nearest = name
    
    return nearest, min_dist

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    for _ in range(5):
        robot.step(timestep)

def check_heading_to_target(target_pos):
    pos1 = get_position()
    
    dist_to_target = distance(pos1, target_pos)
    
    if dist_to_target < 0.8:
        if check_any_obstacle():
            return False, 999
        return True, 0
    
    dx_desired = target_pos[0] - pos1[0]
    dy_desired = target_pos[1] - pos1[1]
    desired_heading = math.atan2(dy_desired, dx_desired)
    
    for _ in range(30):
        if check_any_obstacle():
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
        return False, 999
    
    actual_heading = math.atan2(dy_actual, dx_actual)
    heading_error = normalize_angle(desired_heading - actual_heading)
    
    is_aligned = abs(heading_error) < math.radians(15)
    return is_aligned, heading_error

def turn_until_facing_target(target_pos):
    max_attempts = 25
    attempt = 0
    
    while attempt < max_attempts:
        attempt += 1
        
        is_aligned, heading_error = check_heading_to_target(target_pos)
        
        if is_aligned:
            return True
        
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
    
    return False

def handle_door_arrival(node, is_final_destination=False):
    """Handle arrival at a door - only verify if it's the final destination"""
    print(f"Reached: {node.name}")
    
    if not isinstance(node, DoorNode):
        return
    
    print(f"At door for Room {node.room_number}")
    
    # Only verify if this is the final destination
    if is_final_destination:
        # Pause and stabilize
        for _ in range(20):
            robot.step(timestep)
        
        # Verify room number
        verified = verify_room_at_door(node)
        
        if not verified:
            print(f"  ⚠ Warning: Could not verify Room {node.room_number}")
    else:
        print(f"  (Passing through, not verifying)")

def drive_to_node(target_pos, node_name, is_final_destination=False):
    step = 0
    max_steps = 2000
    last_check = 0
    prev_dist = float('inf')
    stuck_counter = 0
    
    while robot.step(timestep) != -1 and step < max_steps:
        step += 1
        
        current_pos = get_position()
        dist = distance(current_pos, target_pos)
        
        # Check if arrived - CLOSER THRESHOLD
        if dist < 0.35:  # Changed from 0.45 to 0.35
            stop()
            handle_door_arrival(graph.nodes[node_name], is_final_destination)
            return True
        
        # DETECT if we're getting farther away (overshot)
        # Only trigger if we're EXTREMELY close and moving away
        if dist > prev_dist and dist < 0.6:  # Changed from 0.8 to 0.6
            print(f"  Detected overshoot at {dist:.2f}m, stopping")
            stop()
            handle_door_arrival(graph.nodes[node_name], is_final_destination)
            return True
        
        # DETECT if stuck/looping
        if abs(dist - prev_dist) < 0.01:
            stuck_counter += 1
            if stuck_counter > 50:
                print(f"  Stuck at {dist:.2f}m from target, stopping")
                stop()
                handle_door_arrival(graph.nodes[node_name], is_final_destination)
                return True
        else:
            stuck_counter = 0
        
        prev_dist = dist
        
        if check_any_obstacle():
            stop()
            
            for _ in range(20):
                left_motor.setVelocity(-1.5)
                right_motor.setVelocity(-1.5)
                robot.step(timestep)
            stop()
            
            turn_until_facing_target(target_pos)
            last_check = step
            continue
        
        if dist > 2.0 and step - last_check > 200:
            last_check = step
            stop()
            
            is_aligned, heading_error = check_heading_to_target(target_pos)
            
            if not is_aligned and abs(heading_error) > math.radians(25):
                turn_until_facing_target(target_pos)
        
        # PROGRESSIVE SPEED REDUCTION
        if dist < 0.8:
            base_speed = 0.5
        elif dist < 1.5:
            base_speed = 1.0
        else:
            base_speed = min(2.5, dist * 1.5)
        
        correction = 0
        
        if dist > 1.0:
            left_wall, right_wall = get_wall_distances()
            
            if (left_wall is not None and right_wall is not None and 
                0.2 < left_wall < 2.5 and 0.2 < right_wall < 2.5):
                
                wall_diff = right_wall - left_wall
                correction = wall_diff * 0.35
                correction = max(-0.5, min(0.5, correction))
        
        left_speed = base_speed - correction
        right_speed = base_speed + correction
        
        left_speed = max(0, min(3.0, left_speed))
        right_speed = max(0, min(3.0, right_speed))
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
    stop()
    return False
# =============================================================================
# MAIN
# =============================================================================
print("="*60)
print("GRAPH NAVIGATION WITH ROOM VERIFICATION")
print("="*60)

graph = load_graph_from_file('map_config.json')

if graph is None:
    print("ERROR: Could not load graph!")
    stop()
else:
    # ===================================================================
    # SET DESTINATION
    # ===================================================================
    DESTINATION = "door_102"
    # Or: DESTINATION = graph.find_door_by_room(101)
    # ===================================================================

    print(f"Destination: {DESTINATION}\n")

    nearest_node, dist_to_nearest = find_nearest_node(graph)

    if dist_to_nearest > 1.0:
        print(f"Going to {nearest_node} first...")
        target_pos = graph.get_position(nearest_node)
        turn_until_facing_target(target_pos)
        drive_to_node(target_pos, nearest_node, is_final_destination=False)

    path = astar(graph, nearest_node, DESTINATION)

    if path is None:
        print(f"No path found to {DESTINATION}!")
    else:
        print(f"Path: {' -> '.join(path)}\n")
        
        # Navigate along path
        for i in range(len(path)-1):
            next_node = path[i+1]
            next_pos = graph.get_position(next_node)
            
            print(f"Going to: {next_node}")
            
            turn_until_facing_target(next_pos)
            
            # Check if this is the final destination
            is_final = (next_node == DESTINATION)
            
            success = drive_to_node(next_pos, next_node, is_final_destination=is_final)
            
            if not success:
                print("Navigation failed!")
                break
            
            for _ in range(30):
                robot.step(timestep)
        
        print(f"\n✓ ARRIVED AT DESTINATION")
        print("="*60)

    stop()