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
        self.wall_normal = [0, 1]  # Default wall normal
    
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
# LOAD GRAPH FROM PARAMETRIC HALLWAYS
# =============================================================================
def load_graph_from_hallways(filename):
    """Load parametric hallway definition and auto-generate graph"""
    graph = NavigationGraph()
    
    controller_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(controller_dir, filename)
    
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except FileNotFoundError:
        print(f"ERROR: Config file not found at {config_path}")
        return None
    
    hallways = config['hallways']
    
    # Step 1: Find all unique corner positions (hallway intersections)
    corners = {}
    
    for hallway in hallways:
        start = tuple(hallway['start'])
        end = tuple(hallway['end'])
        
        # Add start and end as potential corners
        if start not in corners:
            corners[start] = []
        if end not in corners:
            corners[end] = []
        
        corners[start].append(hallway['name'])
        corners[end].append(hallway['name'])
    
    # Step 2: Create corner nodes
    corner_node_map = {}  # Maps position -> node name

    for pos, connected_halls in corners.items():
        # Generate unique corner name based on exact position
        x, y = pos
        
        # Use position coordinates as name (multiplied by 10 to avoid decimals)
        node_name = f"corner_{int(x*10):+d}_{int(y*10):+d}"
        
        corner_node = CornerNode(node_name, pos)
        graph.add_node(corner_node)
        corner_node_map[pos] = node_name
        
        print(f"Created corner: {node_name} at {pos}")
    
    # Step 3: Process each hallway
    for hallway in hallways:
        start_pos = tuple(hallway['start'])
        end_pos = tuple(hallway['end'])
        
        start_node = corner_node_map[start_pos]
        end_node = corner_node_map[end_pos]
        
        # Determine hallway orientation (horizontal or vertical)
        if start_pos[0] == end_pos[0]:
            # Vertical hallway
            orientation = "vertical"
        else:
            # Horizontal hallway
            orientation = "horizontal"
        
        # Create door nodes along this hallway
        door_nodes = []
        
        for door_info in hallway['doors']:
            # USE FULL POSITION from JSON (both X and Y)
            door_pos = tuple(door_info['position'])
            room_number = door_info['room']
            side = door_info['side']
            
            # Determine wall normal based on side
            # Wall normal points FROM door INTO hallway
            # Determine wall normal based on side
            wall_normals = {
                "north": [0, 1],   # FLIPPED - was [0, -1]
                "south": [0, -1],  # FLIPPED - was [0, 1]
                "east": [1, 0],    # FLIPPED - was [-1, 0]
                "west": [-1, 0]    # FLIPPED - was [1, 0]
            }
            
            wall_normal = wall_normals.get(side, [0, 1])
            
            door_name = f"door_{room_number}"
            door_node = DoorNode(door_name, door_pos, room_number=room_number)
            door_node.wall_normal = wall_normal  # Add wall normal
            
            graph.add_node(door_node)
            door_nodes.append(door_name)
            
            print(f"Created door: {door_name} at {door_pos}, side={side}, normal={wall_normal}")
        
        # Step 4: Create edges along hallway
        # Connect: start -> intermediate corners -> doors -> end
        
        # Determine hallway's fixed coordinate
        if orientation == "horizontal":
            fixed_coord = start_pos[1]  # Y is fixed
        else:
            fixed_coord = start_pos[0]  # X is fixed
        
        # Find all corners that lie ON this hallway
        corners_on_hallway = []
        
        if orientation == "horizontal":
            # Check which corners have same Y coordinate
            for corner_pos in corners.keys():
                if abs(corner_pos[1] - fixed_coord) < 0.01:  # Same Y
                    # Check if between start and end X
                    min_x = min(start_pos[0], end_pos[0])
                    max_x = max(start_pos[0], end_pos[0])
                    if min_x <= corner_pos[0] <= max_x:
                        corners_on_hallway.append(corner_node_map[corner_pos])
        else:
            # Check which corners have same X coordinate
            for corner_pos in corners.keys():
                if abs(corner_pos[0] - fixed_coord) < 0.01:  # Same X
                    # Check if between start and end Y
                    min_y = min(start_pos[1], end_pos[1])
                    max_y = max(start_pos[1], end_pos[1])
                    if min_y <= corner_pos[1] <= max_y:
                        corners_on_hallway.append(corner_node_map[corner_pos])
        
        # Sort corners and doors by position along hallway
        if orientation == "horizontal":
            corners_on_hallway.sort(key=lambda n: graph.nodes[n].position[0])
            door_nodes.sort(key=lambda d: graph.nodes[d].position[0])
        else:
            corners_on_hallway.sort(key=lambda n: graph.nodes[n].position[1])
            door_nodes.sort(key=lambda d: graph.nodes[d].position[1])
        
        # Merge corners and doors, maintaining order
        all_nodes = []
        c_idx = 0
        d_idx = 0
        
        while c_idx < len(corners_on_hallway) or d_idx < len(door_nodes):
            if c_idx >= len(corners_on_hallway):
                all_nodes.append(door_nodes[d_idx])
                d_idx += 1
            elif d_idx >= len(door_nodes):
                all_nodes.append(corners_on_hallway[c_idx])
                c_idx += 1
            else:
                corner_node = graph.nodes[corners_on_hallway[c_idx]]
                door_node = graph.nodes[door_nodes[d_idx]]
                
                if orientation == "horizontal":
                    if corner_node.position[0] < door_node.position[0]:
                        all_nodes.append(corners_on_hallway[c_idx])
                        c_idx += 1
                    else:
                        all_nodes.append(door_nodes[d_idx])
                        d_idx += 1
                else:
                    if corner_node.position[1] < door_node.position[1]:
                        all_nodes.append(corners_on_hallway[c_idx])
                        c_idx += 1
                    else:
                        all_nodes.append(door_nodes[d_idx])
                        d_idx += 1
        
        # Connect consecutive nodes
        for i in range(len(all_nodes) - 1):
            graph.add_edge(all_nodes[i], all_nodes[i+1])
            print(f"Connected: {all_nodes[i]} -> {all_nodes[i+1]}")
            
    print(f"\nGenerated graph with {len(graph.nodes)} nodes")
    
    
    return graph
    
# =============================================================================
# PATHFINDING - A*
# ================================================================F=============
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
    """Verify room number using OCR - room number is on door"""
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
        # Convert to PIL Image
        image_array = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
        image_rgb = image_array[:, :, [2, 1, 0]]
        pil_image = Image.fromarray(image_rgb)
        
        # Save full image for debugging
        debug_dir = os.path.dirname(os.path.abspath(__file__))
        full_image_path = os.path.join(debug_dir, f"camera_full_{expected_room}.png")
        pil_image.save(full_image_path)
        print(f"  Saved: {full_image_path}")
        
        # Use larger crop (80% of image)
        crop_width = int(width * 0.8)
        crop_height = int(height * 0.8)
        left = (width - crop_width) // 2
        top = (height - crop_height) // 2
        
        cropped = pil_image.crop((left, top, left + crop_width, top + crop_height))
        
        # Save cropped image
        crop_image_path = os.path.join(debug_dir, f"camera_crop_{expected_room}.png")
        cropped.save(crop_image_path)
        print(f"  Saved: {crop_image_path}")
        
        # Try OCR on cropped image
        print("  Running OCR...")
        text = pytesseract.image_to_string(cropped, config='--psm 6')
        print(f"  OCR text: '{text}'")
        
        # Extract only digits
        detected_numbers = ''.join(filter(str.isdigit, text))
        print(f"  Extracted numbers: '{detected_numbers}'")
        
        # If crop didn't work, try full image
        if not detected_numbers:
            print("  Trying full image...")
            text_full = pytesseract.image_to_string(pil_image, config='--psm 6')
            print(f"  Full OCR text: '{text_full}'")
            detected_numbers = ''.join(filter(str.isdigit, text_full))
            print(f"  Extracted numbers: '{detected_numbers}'")
        
        if detected_numbers:
            detected_room = int(detected_numbers)
            if detected_room == expected_room:
                print(f"  ✓ Verified: Room {detected_room}")
                return True
            else:
                print(f"  ✗ Wrong room! Expected {expected_room}, saw {detected_room}")
                return False
        else:
            print(f"  ✗ No room number detected")
            return False
            
    except Exception as e:
        print(f"  OCR error: {e}")
        return False

def verify_room_at_door(door_node):
    """Turn to face door using wall normal and verify room number"""
    print(f"  Verifying Room {door_node.room_number}...")
    print(f"  DEBUG: Door position: {door_node.position}")
    print(f"  DEBUG: Wall normal: {door_node.wall_normal}")
    
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
    
    print(f"  DEBUG: Current heading: {math.degrees(current_heading):.1f}°")
    
    # USE WALL NORMAL to determine which way to face
    wall_normal = door_node.wall_normal
    
    # Face opposite of wall normal (toward the door)
    face_direction_x = -wall_normal[0]
    face_direction_y = -wall_normal[1]
    
    target_heading = math.atan2(face_direction_y, face_direction_x)
    
    print(f"  DEBUG: Target heading: {math.degrees(target_heading):.1f}°")
    print(f"  DEBUG: Face direction: [{face_direction_x}, {face_direction_y}]")
    
    # Calculate turn needed
    angle_to_turn = normalize_angle(target_heading - current_heading)
    
    print(f"  Turning {math.degrees(angle_to_turn):.1f}° to face door...")
    
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
    
    # Back up for better camera angle
    print("  Backing up for better view...")
    backup_steps = 20
    
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
    
    # Move forward back to door position
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
        
        # Check if arrived
        if dist < 0.35:
            stop()
            handle_door_arrival(graph.nodes[node_name], is_final_destination)
            return True
        
        # Detect overshoot
        if dist > prev_dist and dist < 0.6:
            print(f"  Detected overshoot at {dist:.2f}m, stopping")
            stop()
            handle_door_arrival(graph.nodes[node_name], is_final_destination)
            return True
        
        # Detect if stuck
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
        
        # Progressive speed reduction
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
print("PARAMETRIC HALLWAY NAVIGATION")
print("="*60)

graph = load_graph_from_hallways('H_Hallway.json')

if graph is None:
    print("ERROR: Could not load graph!")
    stop()
else:
    # ===================================================================
    # SET DESTINATION
    # ===================================================================
    DESTINATION = "corner_-250_+125"
    # Or: DESTINATION = graph.find_door_by_room(101)
    # ===================================================================

    print(f"\nDestination: {DESTINATION}\n")

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