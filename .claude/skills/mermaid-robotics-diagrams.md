---
name: Mermaid Robotics Diagrams
description: Pre-built Mermaid diagram patterns for robotics and AI content
---

# Robotics Mermaid Diagram Patterns

## Robot System Architecture
```mermaid
flowchart TB
    subgraph Perception["🔍 Perception Layer"]
        CAM[Camera]
        LIDAR[LiDAR]
        IMU[IMU Sensor]
    end
    subgraph Processing["🧠 Processing Layer"]
        SLAM[SLAM Module]
        PLAN[Motion Planner]
    end
    subgraph Actuation["⚡ Actuation Layer"]
        MOT[Motors]
        GRIP[Grippers]
    end
    Perception --> Processing --> Actuation
ROS2 Node Communication
mermaid

flowchart LR
    A[sensor_node] -->|/camera/image| B[perception_node]
    B -->|/detected_objects| C[planning_node]
    C -->|/cmd_vel| D[control_node]
Reinforcement Learning Loop
mermaid

flowchart LR
    ENV[Environment] -->|State| AGENT[Agent]
    AGENT -->|Action| ENV
    ENV -->|Reward| AGENT
Sim-to-Real Pipeline
mermaid

flowchart LR
    SIM[Simulator] --> TRAIN[Train] --> DR[Domain Randomization] --> DEPLOY[Deploy] --> ROBOT[Real Robot]
