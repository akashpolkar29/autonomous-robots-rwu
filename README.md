# Autonomous Robots — RWU

**Course:** Autonomous Robots  
**Institution:** Ravensburg-Weingarten University of Applied Sciences (RWU)  
**Program:** M.Sc. Mechatronics  
**Author:** Akash Madhukar Polkar

---

## About

This is a copy of my coursework from the Autonomous Robots subject at RWU, originally hosted on the university GitLab. The course involves hands-on work with a **real TurtleBot** using ROS2 and Docker — this repo is actively being updated as the course progresses.

---

## Repo Structure

```text
├── docker/                  # Dockerfile and docker-compose setup
├── src/
│   ├── stage_1/             # Drive to wall, wall following
│   ├── stage_2/             # Autorace, rainbow circle, TurtleSim
│   ├── stage_3/             # Collision avoidance, custom interfaces
│   └── stage_4/             # Mapping, navigation, waypoint following with TIAGo robot
└── my_bashrc                # Shell config used in the workspace
```

### Stage Overview

* **Stage 1:** Implemented basic robot behaviors including driving to a wall and wall-following using sensor feedback.
* **Stage 2:** Worked on autonomous race challenges, rainbow circle trajectories, and TurtleSim-based exercises.
* **Stage 3:** Developed collision avoidance algorithms and created custom ROS2 interfaces for communication between nodes.
* **Stage 4:** Focused on autonomous navigation using the TIAGo robot, including:

  * Environment mapping and map generation
  * Localization and navigation using the ROS2 navigation stack
  * Waypoint/goal-point following
  * Path planning and autonomous movement between target locations
  * Integration and testing in simulation and robot environments

```

---

## Status

> 🔄 **Currently in progress** — actively working on Stage 3 and beyond.

---

## Dependencies

- ROS2
- Docker / Docker Compose
- TurtleBot3