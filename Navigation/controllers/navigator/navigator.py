# hallway_navigator.py

from controller import Robot
import networkx as nx
import math

class HallwayNavigator:
    def __init__(self):
        # Initialize robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Motors (Pioneer 3-DX uses these names)
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # GPS
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        # Compass
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        # Create hallway graph
        self.graph = self.create_hallway_graph()
        
        # Parameters
        self.max_speed = 5.0
        self.waypoint_threshold = 0.3
        
    def create_hallway_graph(self):
        """Create square hallway with 4 sides"""
        G = nx.Graph()
        
        # Based on your scan: X from -9.61 to 2.30 (12m wide)
        # Assuming square, so Z also goes about 12m
        
        # Four corners of the square
        G.add_node('corner_sw', pos=(-9.61, -6.0))  # Southwest
        G.add_node('corner_se', pos=(2.30, -6.0))   # Southeast  
        G.add_node('corner_ne', pos=(2.30, 6.0))    # Northeast
        G.add_node('corner_nw', pos=(-9.61, 6.0))   # Northwest
        
        # Add points along each hallway (for smoother navigation)
        # South hallway
        G.add_node('south_1', pos=(-6.0, -6.0))
        G.add_node('south_2', pos=(-3.0, -6.0))
        G.add_node('south_3', pos=(0.0, -6.0))
        
        # East hallway
        G.add_node('east_1', pos=(2.30, -3.0))
        G.add_node('east_2', pos=(2.30, 0.0))
        G.add_node('east_3', pos=(2.30, 3.0))
        
        # North hallway
        G.add_node('north_1', pos=(0.0, 6.0))
        G.add_node('north_2', pos=(-3.0, 6.0))
        G.add_node('north_3', pos=(-6.0, 6.0))
        
        # West hallway
        G.add_node('west_1', pos=(-9.61, 3.0))
        G.add_node('west_2', pos=(-9.61, 0.0))
        G.add_node('west_3', pos=(-9.61, -3.0))
        
        # Connect South hallway
        G.add_edge('corner_sw', 'south_1')
        G.add_edge('south_1', 'south_2')
        G.add_edge('south_2', 'south_3')
        G.add_edge('south_3', 'corner_se')
        
        # Connect East hallway
        G.add_edge('corner_se', 'east_1')
        G.add_edge('east_1', 'east_2')
        G.add_edge('east_2', 'east_3')
        G.add_edge('east_3', 'corner_ne')
        
        # Connect North hallway
        G.add_edge('corner_ne', 'north_1')
        G.add_edge('north_1', 'north_2')
        G.add_edge('north_2', 'north_3')
        G.add_edge('north_3', 'corner_nw')
        
        # Connect West hallway
        G.add_edge('corner_nw', 'west_1')
        G.add_edge('west_1', 'west_2')
        G.add_edge('west_2', 'west_3')
        G.add_edge('west_3', 'corner_sw')
        
        return G
    
    def get_position(self):
        """Get robot position"""
        pos = self.gps.getValues()
        return pos[0], pos[2]  # x, z
    
    def get_heading(self):
        """Get robot heading"""
        north = self.compass.getValues()
        return math.atan2(north[0], north[2])
    
    def find_nearest_node(self):
        """Find closest waypoint"""
        x, z = self.get_position()
        min_dist = float('inf')
        nearest = None
        
        for node, data in self.graph.nodes(data=True):
            nx, nz = data['pos']
            dist = math.sqrt((x - nx)**2 + (z - nz)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        print(f"Robot at ({x:.2f}, {z:.2f}) - nearest: {nearest}")
        return nearest
    
    def navigate_to_waypoint(self, target_x, target_z):
        """Move to waypoint"""
        x, z = self.get_position()
        heading = self.get_heading()
        
        # Calculate direction
        dx = target_x - x
        dz = target_z - z
        desired_heading = math.atan2(dx, dz)
        
        # Heading error
        error = desired_heading - heading
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        # Distance
        distance = math.sqrt(dx**2 + dz**2)
        
        # Control
        forward = min(self.max_speed, distance * 2.0)
        turn = error * 1.5
        
        left = forward - turn
        right = forward + turn
        
        # Clamp
        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))
        
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
        
        return distance < self.waypoint_threshold
    
    def navigate(self, goal_node):
        """Navigate to goal"""
        print(f"\n{'='*50}")
        print(f"Navigating to: {goal_node}")
        print(f"{'='*50}\n")
        
        # Wait for GPS to initialize
        print("Waiting for GPS...")
        for i in range(10):
            self.robot.step(self.timestep)
        
        start = self.find_nearest_node()
        
        try:
            path = nx.shortest_path(self.graph, start, goal_node)
            print(f"Path: {' → '.join(path)}\n")
        except:
            print("No path found!")
            return
        
        for i, node in enumerate(path):
            wx, wz = self.graph.nodes[node]['pos']
            print(f"[{i+1}/{len(path)}] Going to {node} at ({wx}, {wz})")
            
            while self.robot.step(self.timestep) != -1:
                if self.navigate_to_waypoint(wx, wz):
                    print(f"✓ Reached {node}")
                    break
        
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        print("\n✓ Navigation complete!\n")

# Main
def main():
    nav = HallwayNavigator()
    
    print("\nAvailable waypoints:")
    for node in sorted(nav.graph.nodes()):
        print(f"  - {node}")
    
    # TEST: Navigate around the square
    nav.navigate('corner_ne')  # Go to northeast corner
    
if __name__ == "__main__":
    main()