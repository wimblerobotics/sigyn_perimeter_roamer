# AI_CONTEXT.md — sigyn_perimeter_roamer

This file provides context for AI coding assistants working in this repository.

## Package Purpose

`sigyn_perimeter_roamer` is a sophisticated ROS 2 package that uses Behavior Trees to 
intelligently navigate a house robot through different types of spaces (rooms, hallways, 
doorways) while avoiding obstacles and managing battery levels. The system is specifically 
designed for the Sigyn robot (cylindrical, ~0.44m radius) navigating residential environments 
with narrow doorways and hallways.

## Repository Structure

```
sigyn_perimeter_roamer/
├── AI_CONTEXT.md              # This file
├── CMakeLists.txt             # Build configuration (ament_cmake)
├── package.xml                # ROS 2 package manifest
├── README.md                  # User-facing documentation
├── BEHAVIORTREE_CONDITIONS.md # Behavior tree condition nodes documentation
├── REACTIVE_NAVIGATION.md     # Reactive navigation system documentation
├── WAYPOINT_CAPTURE_README.md # Waypoint capture tool documentation
├── bt_xml/                    # Behavior tree XML definitions
│   └── perimeter_roamer_tree.xml
├── config/                    # Configuration files
│   └── [navigation params, space thresholds]
├── data/                      # Runtime data (SQLite databases)
│   └── wall_database.db       # House wall geometry database
├── documentation/             # Additional documentation and diagrams
├── include/sigyn_perimeter_roamer/
│   └── bt_nodes.hpp           # Custom behavior tree node declarations
├── launch/
│   ├── perimeter_roamer_launch.py        # Main launch file
│   ├── perimeter_roamer_sim_launch.py    # Simulation launch
│   ├── patrol_using_waypoints_launch.py  # Waypoint-based patrol
│   └── capture_waypoints_launch.py       # Waypoint capture tool
├── scripts/
│   ├── battery_simulator.py   # Battery state simulator for testing
│   └── capture_waypoints.py   # Interactive waypoint capture tool
├── src/
│   ├── perimeter_roamer_node.cpp  # Main node implementation
│   └── bt_nodes.cpp               # Custom behavior tree node implementations
└── test/                      # Unit and integration tests
```

## Core Components

### Perimeter Roamer Node
Main ROS 2 node that orchestrates the entire system. Manages:
- Behavior tree execution and lifecycle
- Integration with Nav2 navigation stack
- LIDAR data processing and space classification
- Wall database queries for localization enhancement
- Battery state monitoring and charging behavior

### Behavior Tree System
Uses BehaviorTree.CPP v3 for robust decision-making with hierarchical structure:

**Main Branches:**
1. **Battery Check Sequence** - Monitors battery and returns to charging when needed
2. **Patrol Sequence** - Executes intelligent navigation based on space type

### Space Classification System
Analyzes LIDAR data and wall database to classify current space:
- **Room**: Large open areas (> 2m width) - patrol edges at 1m+ from walls
- **Hallway**: Corridor-like spaces - stay centered
- **Doorway**: Narrow passages near doorframe width - careful alignment
- **Very Narrow**: Extremely constrained spaces - ultra-cautious movement

### Wall Database
SQLite database containing house wall geometry:
- Wall segments with start/end coordinates
- Room associations and connectivity
- Used for enhanced localization and path planning
- Enables space-aware navigation decisions

## Custom Behavior Tree Nodes

### Condition Nodes
- **CheckBatteryState**: Monitors battery percentage and triggers charging
- **CheckLidarHealth**: Validates LIDAR data quality (checks for invalid readings)
- **IsRoom/IsHallway/IsDoorway/IsVeryNarrow**: Space type classification checks

### Action Nodes
- **ClassifySpace**: Analyzes LIDAR + wall database to determine space type
- **NavigateToPose**: Interface to Nav2 for executing movement commands
- **ReturnToCharger**: Navigate back to charging station (future feature)

## Key Features

- **Intelligent Space-Aware Navigation**: Different strategies per space type
- **Wall Database Integration**: Enhanced localization and navigation planning
- **Battery Management**: Automatic low-battery handling and charging
- **LIDAR Health Monitoring**: Detects sensor failures or data quality issues
- **Nav2 Integration**: Uses Navigation2 stack for path planning/execution
- **Robust Error Handling**: Behavior tree structure enables graceful recovery
- **Waypoint System**: Capture and patrol predefined waypoints (optional mode)
- **Simulation Support**: Works with Gazebo for testing

## Dependencies

- ROS 2 Jazzy
- Navigation2 (Nav2) stack
- BehaviorTree.CPP v3 (behaviortree_cpp_v3)
- SQLite3 for wall database
- TF2 for coordinate transformations
- Standard geometry and sensor messages

## Launch Patterns

The package supports multiple launch modes:

```bash
# Main perimeter roaming mode
ros2 launch sigyn_perimeter_roamer perimeter_roamer_launch.py

# Simulation with Gazebo
ros2 launch sigyn_perimeter_roamer perimeter_roamer_sim_launch.py

# Waypoint-based patrol mode
ros2 launch sigyn_perimeter_roamer patrol_using_waypoints_launch.py

# Capture waypoints interactively
ros2 launch sigyn_perimeter_roamer capture_waypoints_launch.py
```

## Configuration

Key parameters (typically in config directory):
- Space classification thresholds (doorway width, hallway width, etc.)
- Battery charging thresholds (critical level, return-to-charge level)
- LIDAR health monitoring parameters (invalid reading threshold)
- Navigation parameters (goal tolerance, speed limits per space type)
- Wall database path

## Coding Standards

- **Style**: Google C++ Style Guide
- **License**: Apache-2.0
- **Headers**: All source files include SPDX-License-Identifier and copyright
- **Naming**: PascalCase for classes, snake_case for functions/variables
- **Documentation**: Doxygen-style comments for public APIs

## Integration with Sigyn2

This package is part of the Sigyn2 robotics platform and integrates with:
- Nav2 navigation stack for path planning and obstacle avoidance
- Robot hardware sensors (LIDAR, battery monitor)
- AMCL or other localization systems
- TF tree for robot pose tracking

The package is designed to be launched independently for autonomous perimeter 
patrol functionality, rather than being part of the main robot bringup.

## Robot Specifications

- **Platform**: Sigyn house robot
- **Shape**: Cylindrical
- **Radius**: ~0.44m (~0.88m diameter)
- **Navigation**: Differential drive
- **Sensors**: LIDAR (360° scan), IMU, battery monitor
- **Environment**: Indoor residential (narrow doorways, hallways, rooms)

## Testing Tools

### Battery Simulator
`scripts/battery_simulator.py` - Publishes simulated battery state for testing 
battery management behavior without real hardware.

### Waypoint Capture
`scripts/capture_waypoints.py` - Interactive tool for recording robot positions 
as waypoints for patrol routes. Saves to SQLite database.

## Wall Database Schema

The wall database (data/wall_database.db) typically contains:
- **walls** table: wall_id, start_x, start_y, end_x, end_y, room_id
- **rooms** table: room_id, name, type
- **connections** table: room1_id, room2_id, doorway_location

Used for:
- Determining current room/space type
- Planning room-to-room transitions
- Identifying doorway locations and orientations
