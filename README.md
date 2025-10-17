# 🤖 2-Link Robotic Arm Trajectory Tracking (MATLAB)

This project simulates a **2-link planar robotic arm** that follows a **circular path** using a simple **PD controller**.  
It shows both the **desired** and **actual** motion of the robot in MATLAB through plots and animation.

---

## 🎯 Project Goal

To control a 2-link robotic arm so that its end-effector follows a desired circular trajectory using **inverse kinematics** and **PD control**.

---

## ⚙️ How It Works

1. A circular trajectory is defined as the **desired end-effector path**.  
2. **Inverse kinematics** calculates the required joint angles.  
3. A **PD controller** generates the torques needed to follow this path.  
4. The arm motion is simulated using a simple dynamic model.  
5. MATLAB visualizes the result with two figures:
   - The end-effector trajectory (desired vs actual)
   - The arm motion animation

---

## 📈 Results

### 🔹 Trajectory Tracking
Blue: Desired path  
Red dashed: Actual path  

![End-Effector Trajectory](docs)

### 🔹 Robot Motion
The arm animation shows how the joints move to follow the circular path.  

![Robot Arm Animation](docs/robot_arm_animation.png)

---

## ▶️ How to Run

1. Open MATLAB.  
2. Run the script:
   ```matlab
   main.m
