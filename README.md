# HIT – Intelligent Autonomous Vehicle Software Development

This repository contains the infrastructure and ROS 2 software stack developed as part of the **Intelligent Autonomous Vehicle Software Development** course at HIT.

The project demonstrates a **containerized ROS 2 system** integrated with a **web-based control and visualization interface**, following clean architectural separation and ROS 2 best practices.

---

# Project Overview

The system implements a modular ROS 2 environment designed to support:
- Autonomous vehicle logic
- Simulation components
- Web-based control and visualization via ROS 2 bridging

The focus is on **system architecture, ROS graph design, and interoperability**, rather than monolithic or GUI-bound execution.

---

#  System Architecture

The system is composed of two main runtime services and one external client layer.

---

# High-Level Architecture Diagram

```mermaid
flowchart LR
    Browser[Web Client<br/>Canvas / UI] -->|WebSocket (roslibjs)| Rosbridge
    Rosbridge -->|DDS| RosCore
    RosCore -->|DDS| Rosbridge
    Rosbridge -->|WebSocket| Browser


---

# Components

Web Client

- Runs in a browser

- Uses roslibjs

- Publishes and subscribes to ROS 2 topics

- Renders simulation state on a canvas-based UI

rosbridge

- ROS 2 WebSocket server

- Translates WebSocket messages to DDS

- Acts as a bridge between ROS 2 and non-ROS environments

ros2-core

- Main ROS 2 runtime

- Hosts simulation, control, and logic nodes

- Maintains the authoritative ROS graph

---


# ROS 2 Logical Graph

flowchart TB
    WebUI[Web UI<br/>roslibjs]
    RosbridgeNode[/rosbridge_websocket/]

    TalkerNode[/demo_nodes_cpp/talker/]
    ListenerNode[/demo_nodes_cpp/listener/]

    CmdVel[/cmd_vel topic/]
    Chatter[/chatter topic/]

    WebUI -->|publish| CmdVel
    CmdVel --> RosbridgeNode
    RosbridgeNode --> TalkerNode

    TalkerNode -->|publish| Chatter
    Chatter --> RosbridgeNode
    RosbridgeNode --> WebUI
    Chatter --> ListenerNode

Explanation

- Web client behaves as a logical ROS node
- rosbridge_websocket exposes ROS topics over WebSocket
- Core ROS nodes communicate exclusively via DDS
- No ROS logic is duplicated in the browser


---


# Repository Structure

HIT-Autonomous-Vehicle-Dev/
├── docker/
│   ├── ros2-core/
│   │   └── Dockerfile
│   │
│   └── rosbridge/
│       └── Dockerfile
│
├── src/
│   └── turtle_sim_web/
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/
│
├── docker-compose.yml
├── .env
├── .dockerignore
├── .gitignore
└── README.md

---

# Structure Rationale

- docker/ – Container definitions per responsibility
- src/ – Custom ROS 2 packages mounted into ros2-core
- docker-compose.yml – Single orchestration entry point
- Clear separation between infrastructure and ROS logic

---

# Docker Services

ros2-core
- Base image: ros:jazzy-ros-base
- Responsibilities:
- ROS 2 nodes execution
- Simulation
- Control logic
- Exposes DDS to internal Docker network

rosbridge
- Base image: ros:jazzy-ros-base
- Installed package: rosbridge_server
- Exposes:
  -WebSocket on port 9090

---

# Build and Run

Build and start the system
- docker compose up --build

Access rosbridge
- ws://localhost:9090

---

# Verify ROS graph (inside ros2-core)
docker exec -it ros2-core bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list

---

# Technologies Used

- ROS 2 Jazzy
- DDS
- Docker & Docker Compose
- rosbridge
- WebSocket
- Python
- C++
- JavaScript (roslibjs)

---