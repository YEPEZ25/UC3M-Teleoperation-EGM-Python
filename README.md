# UC3M-Teleoperation-EGM-Python

# 🤖 ABB Robot Control via EGM and Python

This project enables real-time control of an ABB robot using the **EGM (Externally Guided Motion)** protocol through a **Python** script. By establishing a UDP connection, you can send Cartesian targets to the robot and receive feedback on its position and joint states — ideal for teleoperation, trajectory tracking, or external system integration.

---

## 📦 Installation Requirements

Before running the system, ensure you have the following tools and packages installed:

### 🔧 RobotStudio
- RobotStudio 2023 or later
- Virtual controller with **EGM option enabled** (RobotWare with EGM license)
- Configure `EGMDevice` in RobotStudio with IP `127.0.0.1` and port `6510`

### 🐍 Python
- Python 3.8 or newer
- Install dependencies using:

```bash
pip install protobuf numpy
```

### 📁 Required Files
- `egm_pb2.py`: generated from ABB's official `.proto` EGM file
- `teleoperation.py`: main Python script to send Cartesian targets
- `EGM.mod`: RAPID module that starts and manages EGM on the robot

---

## ⚙️ System Overview

The system is composed of **two key components**:

---

## 1️⃣ RAPID Code – `EGM.mod`

This RAPID module configures and runs the EGM session on the robot controller.

### Features:
- Initializes the UDP connection (`EGMSetupUC`)
- Activates EGM Pose streaming mode using `EGMActPose`
- Runs a continuous loop with `EGMRunPose` while `keepRunning = TRUE`

### Key Snippet:
```rapid
PROC main()
  EGMGetId egmID;
  EGMSetupUC ROB_1, egmID, "default", "EGMDevice" \Pose \CommTimeout:=10000;
  EGMActPose egmID \StreamStart \Tool:=myTool \Wobj:=myWobj, ...
  WHILE keepRunning DO
    EGMRunPose egmID, EGM_STOP_HOLD \x \y \z \rx \ry \rz;
    WaitTime 0.01;
  ENDWHILE
  EGMStreamStop egmID;
  EGMReset egmID;
ENDPROC
```

---

## 2️⃣ Python Code – `teleoperation.py`

This Python script connects to the robot via UDP, sends Cartesian target poses, and monitors feedback.

### Features:
- Ensures robot starts only from a safe joint configuration
- Sends a sequence of Cartesian targets using `EgmSensor` messages
- Prints current robot position and joint states in real time
- Verifies that each target is reached before sending the next one

### Safety Logic Example:
```python
if not first_position_sent and current_target == 0:
    if not joints_close(joints, safe_joint_position):
        print("⛔ Joints are not in the safe position. Waiting...")
        continue
    else:
        print("✔ Safe position confirmed. Sending the first target.")
        first_position_sent = True
```

---

## 🚀 How to Use

1. Load the `EGM.mod` module into your robot or RobotStudio controller.
2. Run the `main` procedure in RAPID.
3. Launch the Python script:

```bash
python teleoperation.py
```

4. The robot will start moving through the defined sequence of targets.

---

## 🧠 Additional Notes

- The orientation is fixed using default unit quaternions.
- You can customize the list of targets in the Python script by modifying the `targets` variable.
- The script only sends the **first target** after verifying that the robot joints are in a safe position (J1–J6 near [0, 0, 0, 0, 90, 90]).

---

## 🛠️ Generating `egm_pb2.py`

If you do not have `egm_pb2.py`, you can generate it from ABB’s `.proto` file as follows:

1. Download ABB’s official `egm.proto` file from their SDK.
2. Run the following command:

```bash
protoc --python_out=. egm.proto
```

This will create the `egm_pb2.py` file required to encode/decode EGM messages.

---