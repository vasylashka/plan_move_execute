# xArm PyBullet Simulator - Complete Services & Topics Guide

## **TOPICS (Published by the Simulator)**

Topics are read-only data streams that your application subscribes to. The simulator publishes these at a regular rate.

---

### **1. `/xarm/joint_states`**

**Message Type:** `sensor_msgs/msg/JointState`

**Purpose:** Reports the current angles, velocities, and efforts of all robot joints.

**Message Structure:**
```yaml
header:
  seq: 0
  stamp: 
    sec: 1234567890
    nsec: 123456789
frame_id: "world"
name: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
position: [0.1, -0.2, 0.05, 0.3, -0.1, 0.2, 0.0]    # Current angle in radians
velocity: [0.0, 0.0, 0.0, 0. 0, 0.0, 0.0, 0.0]       # Angular velocity in rad/s
effort: [0.0, 0.0, 0.0, 0. 0, 0.0, 0.0, 0.0]         # Torque in Nm
```

**How to Use:**
```bash
# Subscribe to joint states
ros2 topic echo /xarm/joint_states

# Or in Python:
def joint_states_callback(msg):
    print(f"Joint 1 position: {msg.position[0]} rad")
    print(f"Joint 1 velocity: {msg.velocity[0]} rad/s")

node.create_subscription(JointState, '/xarm/joint_states', joint_states_callback, 10)
```

**Real-world Scenario:**
- Used to display the robot model in RViz
- Used for feedback control
- Used to log robot trajectories
- Used to detect if robot reached target position

---

### **2. `/xarm/robot_states`**

**Message Type:** `xarm_msgs/msg/RobotMsg`

**Purpose:** Comprehensive robot status including TCP pose, joint states, mode, errors, and warnings.

**Message Structure:**
```yaml
header:
  stamp: <current_time>
  frame_id: "world"

# Joint angles
angle: [0.1, -0.2, 0.05, 0.3, -0.1, 0. 2, 0.0]

# TCP position and orientation
# Format: [X(mm), Y(mm), Z(mm), Roll(rad), Pitch(rad), Yaw(rad)]
pose: [206.5, 0.0, 121.3, 3.1416, 0.0, 0.0]

# Robot operating state
state: 1    # 0=unknown, 1=idle, 2=running, 3=paused, 4=error

# Control mode
mode: 0     # 0=position, 1=velocity, 2=teach, etc.

# Command counter
cmdnum: 42

# Error and warning codes
err: 0      # 0=no error, non-zero=error code
warn: 0     # 0=no warning, non-zero=warning code

# Motor brake and enable status
mt_brake: 0
mt_able: 0
```

**How to Use:**
```bash
# Watch robot states
ros2 topic echo /xarm/robot_states

# In Python:
def robot_states_callback(msg):
    if msg.err != 0:
        print(f"ERROR CODE: {msg.err}")
    
    print(f"TCP Position: X={msg.pose[0]}mm, Y={msg.pose[1]}mm, Z={msg. pose[2]}mm")
    print(f"TCP Orientation: Roll={msg.pose[3]}rad, Pitch={msg.pose[4]}rad, Yaw={msg.pose[5]}rad")
    print(f"Robot State: {msg.state} (1=idle, 2=running)")

node.create_subscription(RobotMsg, '/xarm/robot_states', robot_states_callback, 10)
```

**Real-world Scenario:**
- Error handling and monitoring
- Display robot status in control panel
- Check if robot is idle before sending new commands
- Log TCP path for trajectory analysis


---

## **SERVICES (Call-and-Response)**

Services require you to make a request and receive a response. Use `ros2 service call` or call them programmatically.

---

## **Motion Control Services**

### **1. `/xarm/set_servo_angle`**

**Service Type:** `xarm_msgs/srv/MoveJoint`

**Purpose:** Move robot by specifying joint angles (radians).

**Request:**
```yaml
angles: [0.0, -0.5, 1.57, 0.0, 0.0, 0.0, 0.0]  # 7 angles in radians
speed: 0.35              # rad/s (angular velocity)
acc: 0.8                 # rad/s² (angular acceleration)
mvtime: 0                # 0 = auto-calculate time
wait: true               # Block until motion complete
timeout: 5.0             # Timeout in seconds
radius: 0                # Rounding radius
relative: false          # false=absolute, true=relative to current
```

**Response:**
```yaml
ret: 0                   # Return code (0=success, negative=error)
```

**How to Call:**
```bash
# CLI
ros2 service call /xarm/set_servo_angle xarm_msgs/srv/MoveJoint \
  "angles: [0. 1, -0.5, 1.57, 0.0, 0.0, 0.0, 0. 0]
   speed: 0.35
   acc: 0.8
   mvtime: 0
   wait: true
   timeout: 5.0
   radius: 0
   relative: false"
```

```python
# Python
from xarm_msgs.srv import MoveJoint

request = MoveJoint.Request()
request.angles = [0. 0, -0.5, 1.57, 0.0, 0.0, 0. 0, 0.0]
request.speed = 0.35
request.acc = 0. 8
request.mvtime = 0
request.wait = True
request.timeout = 5.0
request.radius = 0
request.relative = False

future = client_set_servo_angle.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()
print(f"Return code: {result.ret}")
```

**Real-world Scenario:**
- Moving to a safe/home position before starting
- Repositioning for the next pick operation
- Avoiding collisions with obstacles
- Example: Move to home position before shutdown

---

### **2. `/xarm/set_position`**

**Service Type:** `xarm_msgs/srv/MoveCartesian`

**Purpose:** Move robot to a Cartesian TCP position and orientation.

**Request:**
```yaml
pose: [206.0, 0.0, 121.0, 3.1416, 0.0, 0.0]  
# [X(mm), Y(mm), Z(mm), Roll(rad), Pitch(rad), Yaw(rad)]
# - X/Y/Z: position in millimeters
# - Roll/Pitch/Yaw: orientation in radians

speed: 100.0             # mm/s (TCP linear speed)
acc: 1000.0              # mm/s² (TCP acceleration)
mvtime: 0                # 0 = auto-calculate
wait: true               # Block until done
timeout: 10.0
radius: 0                # Blend radius between segments
relative: false          # false=absolute, true=relative to current
motion_type: 0           # 0=normal, other values for special modes
is_tool_coord: false     # false=base frame, true=tool frame
```

**Response:**
```yaml
ret: 0
```

**How to Call:**
```bash
# Move to position (206, 0, 121) with gripper pointing down
ros2 service call /xarm/set_position xarm_msgs/srv/MoveCartesian \
  "pose: [206.0, 0.0, 121.0, 3. 1416, 0.0, 0.0]
   speed: 100.0
   acc: 1000.0
   mvtime: 0
   wait: true
   timeout: 10.0
   radius: 0
   relative: false
   motion_type: 0
   is_tool_coord: false"
```

```python
# Python - Example: Pick operation
from xarm_msgs.srv import MoveCartesian

# Move above object
move_above_object = MoveCartesian.Request()
move_above_object.pose = [300.0, 150.0, 200.0, 3.1416, 0.0, 0.0]
move_above_object.speed = 150.0
move_above_object.wait = True

future = client_set_position.call_async(move_above_object)
rclpy.spin_until_future_complete(node, future)

if future.result().ret == 0:
    print("Moved above object successfully")
    # Now lower to grasp
    move_to_grasp = MoveCartesian.Request()
    move_to_grasp.pose = [300.0, 150.0, 50.0, 3.1416, 0.0, 0.0]  # Lowered Z
    move_to_grasp.speed = 50.0  # Slower
    # ... call service again
```

**Real-world Scenario:**
- Pick and place operations
- Precision assembly
- Wiping or cleaning surfaces
- Example: Move TCP to (300, 150, 100) to grasp a part

---

### **3.  `/xarm/set_tool_position`**

**Service Type:** `xarm_msgs/srv/MoveCartesian`

**Purpose:** Move tool flange (end-effector mount point) to a position.

**Request:** Same as `set_position`

**Real-world Scenario:**
- When you need precise control of the tool mount point
- Tool length compensation

---

### **4. `/xarm/move_gohome`**

**Service Type:** `xarm_msgs/srv/MoveHome`

**Purpose:** Send robot to home/zero position.

**Request:**
```yaml
speed: 0.5               # rad/s
acc: 1. 0                 # rad/s²
mvtime: 0
wait: true
timeout: 10.0
```

**Response:**
```yaml
ret: 0
```

**How to Call:**
```bash
ros2 service call /xarm/move_gohome xarm_msgs/srv/MoveHome \
  "speed: 0. 5
   acc: 1. 0
   mvtime: 0
   wait: true
   timeout: 10.0"
```

```python
# Python
from xarm_msgs.srv import MoveHome

request = MoveHome.Request()
request.speed = 0.5
request. wait = True
future = client_move_gohome.call_async(request)
rclpy.spin_until_future_complete(node, future)
```

**Real-world Scenario:**
- Robot initialization
- Safe position before shutdown
- Clearing workspace before next task

---

## **State Query Services**

### **6. `/xarm/get_state`**

**Service Type:** `xarm_msgs/srv/GetInt16`

**Purpose:** Get robot's current state. 

**Request:**
```yaml
# Empty request
```

**Response:**
```yaml
ret: 0
data: 1    # 0=unknown, 1=idle, 2=running, 3=paused, 4=error
message: "State code"
```

**How to Call:**
```bash
ros2 service call /xarm/get_state xarm_msgs/srv/GetInt16 {}
```

```python
# Python
from xarm_msgs.srv import GetInt16

request = GetInt16.Request()
future = client_get_state.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result()

if result.data == 1:
    print("Robot is IDLE - safe to send commands")
elif result.data == 2:
    print("Robot is RUNNING")
elif result.data == 4:
    print("Robot has ERROR")
```

**Real-world Scenario:**
- Check if robot is idle before sending new command
- Detect robot errors
- Implement state machine logic

---

### **7. `/xarm/get_servo_angle`**

**Service Type:** `xarm_msgs/srv/GetFloat32List`

**Purpose:** Get current joint angles. 

**Request:**
```yaml
# Empty
```

**Response:**
```yaml
ret: 0
datas: [0.1, -0.2, 0.05, 0.3, -0.1, 0.2, 0.0]  # 7 angles in radians
message: "datas=[ 0.1, -0. 2, ...  ]"
```

**How to Call:**
```bash
ros2 service call /xarm/get_servo_angle xarm_msgs/srv/GetFloat32List {}
```

```python
# Python
from xarm_msgs.srv import GetFloat32List

future = client_get_servo_angle.call_async(GetFloat32List.Request())
rclpy.spin_until_future_complete(node, future)
angles = future.result().datas
print(f"Joint 1: {angles[0]} rad = {angles[0]*180/3.14159} degrees")
```

**Real-world Scenario:**
- Verify robot reached target position
- Log joint angles for trajectory playback
- Detect joint limit violations

---

### **8.  `/xarm/get_position`**

**Service Type:** `xarm_msgs/srv/GetFloat32List`

**Purpose:** Get current TCP position and orientation.

**Response:**
```yaml
ret: 0
datas: [206. 0, 0.0, 121.0, 3.1416, 0.0, 0.0]  
# [X, Y, Z, Roll, Pitch, Yaw]
message: "datas=[ 206. 0, 0.0, 121.0, ...  ]"
```

**How to Call:**
```bash
ros2 service call /xarm/get_position xarm_msgs/srv/GetFloat32List {}
```

```python
# Python
future = client_get_position.call_async(GetFloat32List. Request())
rclpy.spin_until_future_complete(node, future)
pose = future.result().datas

x, y, z = pose[0], pose[1], pose[2]
roll, pitch, yaw = pose[3], pose[4], pose[5]

print(f"TCP at: ({x}, {y}, {z}) mm")
print(f"Orientation: Roll={roll}, Pitch={pitch}, Yaw={yaw} rad")
```

**Real-world Scenario:**
- Verify gripper is at correct location
- Record current position before moving
- Implement visual servoing with feedback

---

## **Mode and Error Control Services**

### **9. `/xarm/set_mode`**

**Service Type:** `xarm_msgs/srv/SetInt16`

**Purpose:** Set robot control mode.

**Request:**
```yaml
data: 0    # 0=position control, 1=velocity control, etc.
```

**Response:**
```yaml
ret: 0
message: "data=0"
```

---

### **10. `/xarm/set_state`**

**Service Type:** `xarm_msgs/srv/SetInt16`

**Purpose:** Set robot state.

**Request:**
```yaml
data: 0    # 0=stop, 1=running, etc.
```

---

### **11. `/xarm/clean_error`**

**Service Type:** `xarm_msgs/srv/Call`

**Purpose:** Clear any error codes.

**Request:**
```yaml
# Empty
```

**Response:**
```yaml
ret: 0
```

**How to Call:**
```bash
ros2 service call /xarm/clean_error xarm_msgs/srv/Call {}
```

```python
# Python - Clear errors
future = client_clean_error.call_async(Call.Request())
rclpy.spin_until_future_complete(node, future)
print("Errors cleared")
```

**Real-world Scenario:**
- Robot encountered error, operator pressed "clear" button
- After clearing collision, resume operation
- Recovery from emergency stop

---

### **12. `/xarm/clean_warn`**

**Service Type:** `xarm_msgs/srv/Call`

**Purpose:** Clear warnings.

---

## **Gripper Control Services**

### **13. `/xarm/set_gripper_position`**

**Service Type:** `xarm_msgs/srv/GripperMove`

**Purpose:** Control gripper position (open/close).

**Request:**
```yaml
pos: 850               # 0=closed, 850=fully open
wait: true             # Block until gripper stops
timeout: 5.0           # Timeout in seconds
```

**Response:**
```yaml
ret: 0
```

**How to Call:**
```bash
# Close gripper
ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove \
  "pos: 0
   wait: true
   timeout: 5.0"

# Open gripper
ros2 service call /xarm/set_gripper_position xarm_msgs/srv/GripperMove \
  "pos: 850
   wait: true
   timeout: 5.0"
```

```python
# Python - Pick and place example
from xarm_msgs.srv import GripperMove

# Close gripper to grasp
close_req = GripperMove.Request()
close_req.pos = 0       # Fully closed
close_req.wait = True

client_gripper.call_async(close_req)

# ...  move robot ... 

# Open gripper to release
open_req = GripperMove.Request()
open_req.pos = 850      # Fully open

client_gripper.call_async(open_req)
```

**Real-world Scenario:**
- Pick: Close gripper (pos=0) to grasp part
- Place: Open gripper (pos=850) to release part
- Adjust grip strength: Use intermediate pos values

---

### **14. `/xarm/get_gripper_position`**

**Service Type:** `xarm_msgs/srv/GetFloat32`

**Purpose:** Query gripper's current position.

**Response:**
```yaml
ret: 0
data: 425.5            # Current gripper position
message: "data=425.5"
```

**How to Call:**
```python
future = client_get_gripper_position.call_async(GetFloat32.Request())
rclpy.spin_until_future_complete(node, future)
gripper_pos = future.result(). data
print(f"Gripper at: {gripper_pos} (0=closed, 850=open)")
```

---

### **15. `/xarm/set_gripper_speed`**

**Service Type:** `xarm_msgs/srv/SetFloat32`

**Purpose:** Set gripper movement speed.

**Request:**
```yaml
data: 1.0              # Speed value (units depend on gripper type)
```

**Real-world Scenario:**
- Slow speed for delicate objects
- Fast speed for quick cycles

---

## **Speed/Acceleration Services**

### **17-20. Speed and Acceleration Configuration**

```bash
# Set max TCP acceleration
ros2 service call /xarm/set_tcp_maxacc xarm_msgs/srv/SetFloat32 "data: 5000.0"

# Set TCP jerk (smoothness)
ros2 service call /xarm/set_tcp_jerk xarm_msgs/srv/SetFloat32 "data: 10000.0"

# Set joint acceleration
ros2 service call /xarm/set_joint_maxacc xarm_msgs/srv/SetFloat32 "data: 10. 0"

# Set joint jerk
ros2 service call /xarm/set_joint_jerk xarm_msgs/srv/SetFloat32 "data: 20.0"
```

**Real-world Scenario:**
- Slow motion for precision tasks
- Fast motion for cycle time
- Smooth motion for fragile payloads
- Aggressive motion for production


---

## **Summary Table**

| Service/Topic | Message Type | Purpose | Input | Output |
|---|---|---|---|---|
| `/xarm/joint_states` | JointState | Joint angles/velocities | - | position, velocity, effort |
| `/xarm/robot_states` | RobotMsg | Overall robot status | - | pose, state, error, mode |
| `/xarm/set_servo_angle` | MoveJoint | Move by joint angles | angles, speed | ret |
| `/xarm/set_position` | MoveCartesian | Move TCP to position | pose, speed | ret |
| `/xarm/get_position` | GetFloat32List | Query TCP position | - | datas=[X,Y,Z,R,P,Y] |
| `/xarm/move_gohome` | MoveHome | Go to home position | speed | ret |
| `/xarm/set_gripper_position` | GripperMove | Control gripper | pos | ret |
| `/xarm/clean_error` | Call | Clear errors | - | ret |
| `/xarm/set_cgpio_digital` | SetDigitalIO | Set digital output | ionum, value | ret |
| `/xarm/get_cgpio_analog` | GetAnalogIO | Read analog input | ionum | data |
