import os
import math
from controller import Robot

# 1. Initialize the Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Get Device Handles
# Make sure these names "RHip" and "LHip" match your Titan.proto exactly!
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

# 3. Validation Check
if r_hip is None or l_hip is None:
    print("ERROR: Hip devices not found! Check your PROTO names.", flush=True)
else:
    print("--- TITAN CONTROLLER: ONLINE AND CONNECTED ---", flush=True)

# 4. Main Control Loop
while robot.step(timestep) != -1:
    t = robot.getTime()
    
    # Create a smooth swinging motion for the hips
    # Using 1.5 frequency to make the movement visible
    val = math.sin(t * 1.5) * 0.5
    
    r_hip.setPosition(val)
    l_hip.setPosition(-val)
    
    # 5. Forced Output for Colab
    # Prints every 1 second of simulation time
    if int(t * 10) % 10 == 0: 
        print(f"Time: {t:.2f}s | Hips swinging to: {val:.3f}", flush=True)
