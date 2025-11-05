# Intelligent Autonomous Vehicle Software Development 🚗🤖

Welcome to the **Intelligent Autonomous Vehicle Software Development** course repository.  
This repository contains all course materials, code examples, assignments, and projects related to the design, development, and testing of intelligent systems for autonomous vehicles.

---

## 📘 Course Overview

This course focuses on the **software architecture, algorithms, and technologies** behind modern autonomous vehicles.  
Students will gain practical experience in perception, planning, control, and decision-making systems through real-world simulations and projects.

### Key Topics
- Fundamentals of Autonomous Systems
- Sensor Fusion (LiDAR, Radar, Cameras)
- Computer Vision and Object Detection
- Localization and Mapping (SLAM)
- Path Planning and Motion Control
- AI and Machine Learning for Perception and Decision-Making
- ROS (Robot Operating System)
- Simulation Environments (CARLA, Gazebo, etc.)

---

## 🧠 Learning Objectives

By the end of this course, you will be able to:
1. Understand the core components of an autonomous vehicle software stack.
2. Design and implement intelligent driving modules.
3. Integrate perception, planning, and control algorithms.
4. Use modern tools and frameworks for simulation and testing.
5. Apply AI and ML techniques to improve vehicle intelligence.

---

## 🧩 Repository Structure

```
HIT-Autonomous-Vehicle-Dev/
├── backend/
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py                 # FastAPI entry point
│   │   ├── routers/                # API endpoints (e.g., perception, control)
│   │   ├── core/                   # Config, logging, and utilities
│   │   └── models/                 # Data models or database schemas (if used)
│   │
│   ├── tests/
│   │   └── test_main.py
│   │
│   ├── requirements.txt
│   ├── Dockerfile
│   └── README.md
│
├── assignments/                    # Weekly coding exercises
│   └── ...
│
├── projects/                       # Final and midterm projects
│   └── autonomous_car/
│       ├── src/
│       ├── config/
│       └── launch/
│
├── simulations/                    # For ROS2 / CARLA / Gazebo simulations
│   ├── rviz/
│   ├── carla/
│   └── gazebo/
│
├── docs/                           # Documentation and course notes
│   └── syllabus.pdf
│
├── .env
├── .gitignore
├── .dockerignore
├── docker-compose.yml
└── README.md
```

⚙️ Setup Instructions
1. Clone the repository
git clone https://github.com/<your-username>/HIT-Autonomous-Vehicle-Dev.git
cd HIT-Autonomous-Vehicle-Dev

2. Create a virtual environment
python3 -m venv venv
source venv/bin/activate

3. Install dependencies
pip install -r requirements.txt


🧪 Tools & Technologies

Languages: Python, C++, MATLAB
Frameworks: ROS2, OpenCV, TensorFlow/PyTorch
Simulators: CARLA, Gazebo
Version Control: Git + GitHub
Other Tools: NumPy, Pandas, Matplotlib


📚 References

CARLA Simulator
ROS Documentation
OpenCV
Udacity: Self-Driving Car Engineer Nanodegree

🧑‍💻 Author

Shaked Sabag
Networking & Cybersecurity Engineer | Cloud & Software Developer
📧 Contact: sabag975@gmail.com