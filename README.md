# 3D & 2D Robot Arm Kinematics Visualizer (Web & Python)

An interactive, high-fidelity **3D & 2D Robot Arm Visualizer and Simulator** for **Forward Kinematics (FK)** and **Inverse Kinematics (IK)**.

Supports articulated robotic arms (default: 4-DOF, customizable from 2-DOF to 6-DOF) with full 3D interactive controls, real-time Inverse Kinematics solvers, robotic gripper tools, trajectory tracking, and Python code generation.

---

## What's New: 3D Robotics Lab

- **Realistic 3D Articulated Robot Arm** rendered in Three.js (WebGL) with PBR metal materials, dynamic soft shadows, and servo bearing hubs.
- **Color-Coded Links** matching your original Matplotlib script:
  - **Link 1**: Red (`#ef4444`)
  - **Link 2**: Green (`#10b981`)
  - **Link 3**: Blue (`#3b82f6`)
  - **Link 4**: Magenta (`#d946ef`)
- **3D Transform Gizmo (TransformControls)**:
  - Grab the **Red (X), Green (Y), and Blue (Z)** translation arrows directly in 3D space to drag the target.
  - The arm solves 3D Inverse Kinematics at **60 FPS** in real-time to follow your hand!
- **Interactive Robotic Gripper**:
  - Dual-finger servo gripper with opening and closing slider (0% to 100%), red guide laser beam, and rubber gripping pads.
- **3D Camera View Presets**:
  - `Iso` (Isometric 3D perspective)
  - `Front` (Front view)
  - `Side` (Side elevation)
  - `Top` (Top-down view)
  - `Tool` (Focus camera directly on the End-Effector / Gripper)
- **3D Workspace Sphere**:
  - Translucent 3D reachable envelope ($R_{max} = \sum l_i$).
- **3D Autonomous Trajectories**:
  - Traces 3D parametric curves: **3D Helix / Spiral**, **3D Circle**, **3D Figure-8 / Lemniscate**, **3D Pick-and-Place Square**, and **3D Sinusoidal Wave** with 3D glowing motion ribbons.
- **1-Click 2D / 3D Switcher**:
  - Seamlessly toggle between **3D Robotics Lab** and **2D Planar Mode** in the header without losing your current arm state!
- **100% Offline Capable**:
  - Three.js and all control libraries are saved locally in `js/libs/`. Works completely offline with zero installation.

---

## 3D Kinematics Math Formulation

The 3D articulated arm uses an azimuthal-elevation decoupling:
1. **Joint 1 (Base Yaw $\theta_1$)**:
   $$\theta_1 = \text{atan2}(Z, X)$$
   Rotates the entire arm base around the vertical axis so that the arm lies in the vertical plane containing the target.
2. **Radial Distance & Height**:
   $$R = \sqrt{X^2 + Z^2}, \quad Y_{\text{planar}} = Y - h_{\text{base}}$$
3. **Joints 2, 3, 4 (Elevation Pitch Angles $\theta_2, \theta_3, \theta_4$)**:
   The elevation chain solves planar IK in the $(R, Y)$ coordinate plane using **FABRIK**, **CCD**, or **Jacobian DLS**.

---

## How to Run

### Method 1: Web Application (3D & 2D)

- **Option A (Double Click)**:
  Double-click [`run_website.bat`](file:///D:/AIR/run_website.bat) or [`index.html`](file:///D:/AIR/index.html).
- **Option B (Terminal)**:
  ```powershell
  python D:\AIR\server.py
  ```
  Opens `http://localhost:8000/index.html` automatically.

### Method 2: 3D Matplotlib Python Script

Run the 3D Python CLI script:
```powershell
python D:\AIR\kinematics_3d_cli.py
```
Or double-click [`run_3d_cli.bat`](file:///D:/AIR/run_3d_cli.bat).

### Method 3: 2D Matplotlib Python Script

Run the 2D Python CLI script:
```powershell
python D:\AIR\kinematics_cli.py
```
Or double-click [`run_python_cli.bat`](file:///D:/AIR/run_python_cli.bat).

---

## Project Structure

```
D:/AIR/
├── index.html               # Main dashboard with 3D & 2D viewports
├── css/
│   └── style.css            # Dark cyberpunk robotics theme
├── js/
│   ├── libs/
│   │   ├── three.min.js     # Three.js 3D WebGL engine (local)
│   │   ├── OrbitControls.js # 3D camera mouse controls (local)
│   │   └── TransformControls.js # 3D interactive drag gizmo (local)
│   ├── robot.js             # Kinematics engine (FK, FABRIK, CCD, Jacobian DLS)
│   ├── canvas.js            # 2D high-DPI canvas renderer
│   ├── robot3d.js           # 3D articulated robot model, gripper, and 3D IK
│   └── app.js               # Synchronized controller, trajectories, exporter
├── server.py                # Local Python HTTP server
├── kinematics_3d_cli.py     # 3D Matplotlib Python script
├── kinematics_cli.py        # 2D Matplotlib Python script
├── run_website.bat          # 1-click web launcher
├── run_3d_cli.bat           # 1-click 3D Python launcher
├── run_python_cli.bat       # 1-click 2D Python launcher
└── README.md                # Documentation
```
# robotics
