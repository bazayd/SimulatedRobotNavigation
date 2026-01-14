# world_mapper.py - Scans your world automatically

from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

print("\n" + "="*60)
print("SCANNING YOUR WEBOTS WORLD...")
print("="*60 + "\n")

# Get all objects in the world
root = supervisor.getRoot()
children_field = root.getField('children')

walls = []
doors = []

# Scan everything
for i in range(children_field.getCount()):
    node = children_field.getMFNode(i)
    if node is None:
        continue
    
    # Get position
    translation_field = node.getField('translation')
    if not translation_field:
        continue
    
    pos = translation_field.getSFVec3f()
    x, y, z = pos[0], pos[1], pos[2]
    
    # Get name and type
    node_type = node.getTypeName()
    name_field = node.getField('name')
    name = name_field.getSFString() if name_field else f"{node_type}_{i}"
    
    name_lower = name.lower()
    type_lower = node_type.lower()
    
    # Check if wall
    if 'wall' in name_lower or 'wall' in type_lower:
        walls.append({'name': name, 'x': x, 'z': z})
        print(f"Found WALL: {name:30s} at x={x:7.2f}, z={z:7.2f}")
    
    # Check if door
    elif 'door' in name_lower or 'door' in type_lower:
        doors.append({'name': name, 'x': x, 'z': z})
        print(f"Found DOOR: {name:30s} at x={x:7.2f}, z={z:7.2f}")
    
    # Check if solid
    elif node_type == 'Solid':
        print(f"Found {node_type}: {name:30s} at x={x:7.2f}, z={z:7.2f}")

# Find hallway bounds
if walls:
    xs = [w['x'] for w in walls]
    zs = [w['z'] for w in walls]
    
    min_x, max_x = min(xs), max(xs)
    min_z, max_z = min(zs), max(zs)
    
    print("\n" + "="*60)
    print("HALLWAY BOUNDARIES:")
    print(f"  X range: {min_x:.2f} to {max_x:.2f} (width: {max_x-min_x:.2f}m)")
    print(f"  Z range: {min_z:.2f} to {max_z:.2f} (length: {max_z-min_z:.2f}m)")
    print("="*60 + "\n")
    
    # Generate corner waypoints
    print("COPY THIS CODE INTO YOUR NAVIGATOR:")
    print("-"*60)
    print("# Corners")
    print(f"G.add_node('corner_sw', pos=({min_x:.2f}, {min_z:.2f}))")
    print(f"G.add_node('corner_se', pos=({max_x:.2f}, {min_z:.2f}))")
    print(f"G.add_node('corner_ne', pos=({max_x:.2f}, {max_z:.2f}))")
    print(f"G.add_node('corner_nw', pos=({min_x:.2f}, {max_z:.2f}))")
    print()

# Print doors
if doors:
    print("# Doors")
    for door in doors:
        safe_name = door['name'].replace(' ', '_').replace('-', '_')
        print(f"G.add_node('{safe_name}', pos=({door['x']:.2f}, {door['z']:.2f}))")
    print()

print("="*60)
print("SCAN COMPLETE!")
print("="*60)

# Keep running
while supervisor.step(timestep) != -1:
    pass