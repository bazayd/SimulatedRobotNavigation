# Simulated Robot Navigation

A comprehensive robot navigation simulation project built with Webots, featuring multiple navigation algorithms, sensor integration, and advanced robotics capabilities.

## 📋 Overview

This project implements a simulated robot navigation system with various controllers for autonomous navigation, obstacle avoidance, and environmental perception. The simulation uses Webots as the robotics simulator and includes implementations of pathfinding algorithms, collision avoidance, SLAM, and multi-modal input processing.

## 🏗️ Project Structure

```
SimulatedRobotNavigation/
│
├── requirements.txt              # Python dependencies
│
├── controllers/                  # Robot controllers
│   ├── CollisionAvoidance/       # Obstacle avoidance controller
│   ├── OCR/                      # Optical Character Recognition
│   ├── SLAM/                     # Simultaneous Localization and Mapping
│   └── SpeechRecognition/        # Voice command processing
│
├── Navigation/                   # Navigation implementations
│   ├── controllers/
│   │   ├── Navigation_AStar/     # A* pathfinding algorithm
│   │   ├── NavV2/                # Navigation version 2
│   │   └── NavV3/                # Navigation version 3 (most advanced)
│   ├── textures/                 # Visual assets
│   └── worlds/
│       └── Indoors.wbt           # Indoor simulation world
│
├── libraries/                    # Shared libraries
│
├── plugins/                      # Webots plugins
│   ├── physics/
│   ├── remote_controls/
│   └── robot_windows/
│
├── protos/                       # Custom Webots prototypes
│
└── worlds/
    └── Indoors.wbt               # Main simulation world
```

## 🚀 Features

### Navigation Systems
- **A* Pathfinding (NavV1)**: Basic graph-based navigation using A* algorithm with corner nodes
- **NavV2**: Enhanced navigation with configurable map parameters
- **NavV3**: Advanced navigation system with:
  - Support for corner nodes and door nodes
  - Hallway-based navigation
  - JSON configuration files for flexible map definitions
  - Room number identification

### Controller Modules
- **Collision Avoidance**: Reactive obstacle avoidance using distance sensors
  - 16 distance sensors for 360° coverage
  - Configurable sensor weights
  - LED indicators for collision states
  - Dynamic speed adjustment

- **SLAM (Simultaneous Localization and Mapping)**:
  - ROS2 integration for mapping and localization
  - Lidar sensor integration (Sick LMS 291)
  - Odometry tracking
  - Real-time position and orientation updates

- **OCR (Optical Character Recognition)**:
  - Text recognition from robot camera
  - Pytesseract integration

- **Speech Recognition**:
  - Voice command processing
  - PyAudio integration for audio input

## 🛠️ Installation

### Prerequisites
- [Webots R2025a](https://cyberbotics.com/) or later
- Python 3.7+
- (Optional) ROS2 for SLAM features

### Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd SimulatedRobotNavigation
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

   Dependencies include:
   - `SpeechRecognition==3.14.4` - Voice command processing
   - `PyAudio==0.2.14` - Audio input handling
   - `pytesseract==0.3.13` - OCR capabilities
   - `numpy==2.3.4` - Numerical computations
   - `pipwin==0.5.2` - Windows package manager

3. **Additional Setup for OCR**:
   - Install [Tesseract OCR](https://github.com/tesseract-ocr/tesseract)
   - Add Tesseract to your system PATH

4. **For SLAM (optional)**:
   - Install [ROS2](https://docs.ros.org/en/rolling/Installation.html)
   - Source your ROS2 installation

## 🎮 Usage

### Running Simulations

1. **Open Webots**:
   - Launch Webots
   - Open the world file: `worlds/Indoors.wbt`

2. **Select a Controller**:
   - In the Scene Tree, select your robot
   - In the Controller field, choose one of:
     - `Navigation_AStar` - Basic A* navigation
     - `NavV2` - Enhanced navigation
     - `NavV3` - Advanced navigation with room support
     - `CollisionAvoidance` - Reactive obstacle avoidance
     - `SLAM` - Mapping and localization

3. **Run the Simulation**:
   - Click the play button in Webots
   - The robot will execute the selected controller

### Navigation Configuration

#### NavV3 Configuration
Edit `Navigation/controllers/NavV3/hallways_config.json` to customize the navigation graph:

```json
{
  "corners": [
    {"name": "corner_se", "position": [1.0, -3.0]},
    {"name": "corner_ne", "position": [1.0, 6.0]}
  ],
  "doors": [
    {"name": "room_101", "position": [0.5, 2.0], "room_number": "101"}
  ],
  "hallways": [
    {"from": "corner_se", "to": "corner_ne"}
  ]
}
```

## 📊 Navigation Algorithms

### A* Pathfinding
The A* algorithm implementation uses:
- **Heuristic**: Euclidean distance
- **Node Types**: Corner nodes (hallway intersections)
- **Path Planning**: Shortest path between waypoints

### Collision Avoidance
The collision avoidance system:
- Uses 16 distance sensors arranged around the robot
- Implements weighted steering based on obstacle proximity
- Maintains a minimum distance of 1.0 meter from obstacles
- Provides visual feedback through LED indicators

## 🤖 Robot Configuration

The simulated robot (Pioneer 3-DX compatible) includes:
- **Sensors**:
  - 16 distance sensors (ultrasonic/IR)
  - Lidar (Sick LMS 291) for SLAM
  - Camera for OCR
  - Microphone for speech recognition
- **Actuators**:
  - Differential drive wheels
  - LED indicators
- **Parameters**:
  - Max speed: 5.24 rad/s
  - Wheel radius: 0.0975 m
  - Wheel distance: 0.33 m

## 🗺️ World Environment

The `Indoors.wbt` world includes:
- 24m × 24m floor space
- Multiple rooms and hallways
- Walls and obstacles
- Textured backgrounds
- Appropriate lighting for camera operations

## 🔧 Development

### Adding New Navigation Nodes
1. Edit the configuration JSON file (e.g., `hallways_config.json`)
2. Add node definitions with positions
3. Define connections between nodes
4. Restart the simulation

### Creating Custom Controllers
1. Create a new folder in `controllers/`
2. Implement the controller class inheriting from `Robot`
3. Define the control logic in the main loop
4. Register the controller in Webots

## 📝 Configuration Files

- `Navigation/controllers/NavV2/map_config.json` - NavV2 map configuration
- `Navigation/controllers/NavV3/hallways_config.json` - NavV3 graph definition
- `Navigation/controllers/NavV3/H_Hallway.json` - H-shaped hallway layout

## 🐛 Troubleshooting

### Common Issues

1. **Import Error for Webots Controller**:
   - Ensure Webots is properly installed
   - Check that the controller is run from within Webots

2. **Speech Recognition Not Working**:
   - Verify PyAudio installation
   - Check microphone permissions
   - On Windows, you may need to use `pipwin install pyaudio`

3. **OCR Not Detecting Text**:
   - Ensure Tesseract is installed and in PATH
   - Check camera positioning and lighting in simulation

4. **SLAM/ROS2 Issues**:
   - Verify ROS2 installation and sourcing
   - Check that the lidar device name matches your world configuration

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- Additional pathfinding algorithms (D*, RRT)
- Enhanced sensor fusion
- Machine learning-based navigation
- Multi-robot coordination
- Real-world robot deployment

## 📄 License

[Specify your license here]

## 👥 Authors

[Specify authors/contributors here]

## 🔗 Resources

- [Webots Documentation](https://cyberbotics.com/doc/guide/index)
- [ROS2 Documentation](https://docs.ros.org/)
- [A* Algorithm Explained](https://en.wikipedia.org/wiki/A*_search_algorithm)

## 📅 Version History

- **v3.0** - Advanced navigation with room support and JSON configuration
- **v2.0** - Enhanced navigation system
- **v1.0** - Basic A* pathfinding implementation

---

*Last Updated: February 2026*
