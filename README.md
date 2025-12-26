# Autonomous Cricket Pitch Curator 🏏🤖

![MATLAB](https://img.shields.io/badge/Language-MATLAB-orange.svg)
![Status](https://img.shields.io/badge/Status-Thesis%20Completed-green.svg)
![Domain](https://img.shields.io/badge/Domain-Robotics%20%26%20Controls-blue.svg)

**A high-fidelity simulation of an autonomous rover designed to maintain cricket pitches with sub-millimeter precision.**

> **Author:** Yashwanth Gowda  
> **Institution:** Arizona State University  
> **Degree:** M.S. in Mechanical & Aerospace Engineering  

---

## 📖 Overview
The condition of a cricket pitch is the single most critical variable in the sport. Traditional manual curation is subjective, labor-intensive, and inconsistent. 

This project proposes and simulates an **Autonomous Pitch Curator**—a skid-steer robotic agent capable of:
1.  **Navigating** the pitch using a coverage-optimized Boustrophedon path.
2.  **Detecting** surface defects (footmarks, divots) using simulated depth sensing.
3.  **Repairing** defects selectively using an "Inspect-and-Repair" actuation protocol.

This simulation proves that autonomous maintenance can achieve **100% surface consistency** while reducing energy consumption by over **90%** compared to traditional continuous rolling.

---

## ⚙️ Key Features
* **Monte Carlo Robustness:** The system is tested against randomized defect generation (stochastic location and size) to ensure algorithmic reliability.
* **Smart Sensor Logic:** Implements a depth thresholding algorithm ($\delta_{th} = -1.0mm$) to distinguish between natural soil texture and damage.
* **Path Optimization:** Utilizes a custom Boustrophedon path planner with **25% overlap** to guarantee zero missed detections.
* **Metrology Metrics:** precise calculation of $R_{rms}$ (Root Mean Square Roughness) and mechanical Duty Cycle.

---

## 📊 Results & Performance
The simulation was validated over 3,500+ time steps with the following performance benchmarks:

| Metric | Result | Description |
| :--- | :--- | :--- |
| **Surface Quality ($R_{rms}$)** | **0.05 mm** | Reduced from initial 0.58 mm (Restored to baseline). |
| **Coverage Efficiency** | **100%** | Full pitch scanned with zero "striping" errors. |
| **Actuator Duty Cycle** | **7.10%** | High-power tamper is idle 93% of the time, saving energy. |
| **Total Energy** | **9,174 J** | Adaptive power profile based on defect density. |

### Visual Validation
**1. Topography Analysis:**
*(Before vs. After: demonstrating the complete removal of random defect clusters)*
![Topography Results](images/topography_results.jpg)

**2. Efficiency Metrics:**
*(Real-time tracking of Roughness reduction and Energy consumption)*
![Metrics Plot](images/metrics_plot.png)

---

## 🚀 How to Run
1.  **Prerequisites:** Ensure you have MATLAB installed (R2020b or later recommended).
2.  **Clone the Repo:**
    ```bash
    git clone [https://github.com/your-username/autonomous-pitch-curator.git](https://github.com/your-username/autonomous-pitch-curator.git)
    cd autonomous-pitch-curator
    ```
3.  **Run the Simulation:**
    * Open `PitchCurator_Final.m` in MATLAB.
    * Run the script.
    * The simulation will generate:
        * A randomized pitch surface.
        * The robot's path execution.
        * Final plots for Topography and Energy Metrics.

---

## 🧠 System Architecture
The robot operates on a Skid-Steer Kinematic model controlled by a central FSM (Finite State Machine):

```mermaid
graph LR
    A[Depth Sensor] -->|Height Map| B(Central Processor)
    C[IMU / GPS] -->|Pose| B
    B -->|PWM| D[Motor Drivers]
    B -->|Trigger| E[Solenoid Tamper]
    D --> F[Robot Motion]
    E --> G[Pitch Surface]
