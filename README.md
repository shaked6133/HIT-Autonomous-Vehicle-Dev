# HIT – Intelligent Autonomous Vehicle Software Development
### Headless ROS 2 Simulation & Web-Interface Integration
<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/index.html)
[![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)


This repository contains a containerized ROS 2 software stack developed for the **Intelligent Autonomous Vehicle Software Development** course at HIT. The project demonstrates a modern "Headless" simulation architecture where ROS 2 handles the physics and logic in a Docker environment, while a decoupled Web Client provides visualization and control.

</div>

## 🏗 Full System Architecture


The following graph illustrates the data flow from the user's browser, through the WebSocket bridge, into the internal ROS 2 DDS network.

```mermaid
flowchart TB
    subgraph Host ["Host Machine (Windows/Linux/Mac)"]
        Browser[Web Browser Interface]
        WebServer[Node.js Static Server :8080]
    end

    subgraph Bridge ["Docker Container: rosbridge"]
        RB[rosbridge_websocket :9090]
    end

    subgraph Core ["Docker Container: ros2-core"]
        direction TB
        TurtleNode[/turtle_node/]
        SimLogic[Simulation Logic]
        TurtleNode --- SimLogic
    end

    %% Communication Flows
    Browser <--> WebServer
    Browser <--> RB
    RB <--> TurtleNode

    %% Topic Details
    TurtleNode -- "/turtle1/pose" --> RB
    RB -- "/turtle1/cmd_vel" --> TurtleNode
```




## 📂 Repository Structure
```
HIT-Autonomous-Vehicle-Dev/
├── docker/
│   ├── ros2-core/          # ROS 2 Jazzy environment & Turtlesim setup
│   └── rosbridge/          # WebSocket bridge server configuration
├── src/
│   └── turtle_sim_web/     # Custom ROS 2 Python package
│       ├── turtle_sim_web/ # Node implementation (turtle_node.py)
│       ├── package.xml     # Package dependencies
│       └── setup.py        # Build entry points for ROS 2
├── web/
│   ├── index.html          # UI with HTML5 Canvas & ROSLibJS
│   ├── app.js              # ROSLibJS Publisher/Subscriber logic
│   └── server.js           # Node.js server for static assets
├── docker-compose.yml      # Orchestration & Volume mounting
└── README.md
```


## 🚀 Getting Started

1. Prerequisites

    Docker Desktop: Ensure WSL2 backend is enabled (for Windows users).

    Git: To clone and manage the repository.

2. Launching the System

From the root directory, execute the following command:

```
docker compose up --build
```

This command builds the custom nodes, handles environment sourcing, and initializes the internal ROS graph.



## 🎮 Operational Control
Teleoperation
Keyboard: Use the Arrow Keys (Up, Down, Left, Right) to drive the turtle on the canvas.

On-Screen Buttons: Use the dedicated UI buttons for manual control.

Services:
Teleport: Use the "Teleport" buttons (TL, TR, BL, BR) 
to instantly move the turtle to the corners via the /teleport_turtle service.


## 🛠 Technical Implementation Details

Windows-Docker Compatibility Fix

To bypass common volume-syncing delays on Windows, the docker-compose.yml utilizes a specialized bootstrap sequence:

    Internal Build: Runs colcon build --merge-install inside the container.

    Manual Path Injection: Explicitly exports the AMENT_PREFIX_PATH to ensure the package is discoverable immediately.

    Subshell Execution: Launches the ROS node in the background using (ros2 run ... &) and maintains the container lifecycle with tail -f /dev/null.


## ROS 2 Interface Specifications
```
Component	    Type	    Name	          Message/Service Type
Publisher	    Topic	  /turtle1/pose	    turtlesim/msg/Pose
Subscriber    Topic	  /turtle1/cmd_vel	geometry_msgs/msg/Twist
Client/Server	Service	/teleport_turtle	std_srvs/srv/SetBool
```


## 🛠 Technologies
    Middleware: ROS 2 Jazzy Jalisco

    Communication: WebSockets, DDS (Data Distribution Service)

    Frontend: HTML5 Canvas, JavaScript, ROSLIBJS

    Infrastructure: Docker & Docker Compose

    Backend: Node.js
</div>