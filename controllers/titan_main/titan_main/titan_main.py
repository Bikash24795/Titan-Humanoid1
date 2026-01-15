import os, sys, math, time
from controller import Robot

# 1. Initialize Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. Setup Sensors with Error Protection
def get_sensor(name):
    s = robot.getDevice(name)
    if s:
        s.enable(timestep)
        return s
    return None

imu = get_sensor('imu')
r_foot = get_sensor('RFootSensor')
l_foot = get_sensor('LFootSensor')
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

print("--- TITAN CONNECTED TO PHYSICS ENGINE ---", flush=True)

while robot.step(timestep) != -1:
    t = robot.getTime()
    if t > 40.0: break # Increased to 40s to ensure we capture data

    # Read Sensors (Default to 0 if not found)
    pitch = imu.getRollPitchYaw()[1] if imu else 0.0
    
    # Calculation: If falling forward (positive pitch), move hips forward
    # Using a slightly higher KP for visible movement
    output = 3.5 * (0.0 - pitch)
    
    # ZMP Calculation (Vertical Force)
    r_val = r_foot.getValues()[2] if r_foot else 0.0
    l_val = l_foot.getValues()[2] if l_foot else 0.0
    total = abs(r_val) + abs(l_val)
    zmp = (r_val - l_val) / total if total > 1.0 else 0.0

    # Actuate Motors
    if r_hip and l_hip:
        r_hip.setPosition(output)
        l_hip.setPosition(output)

    # Log data every 1.5 seconds
    if int(t * 10) % 15 == 0:
        print(f"Time: {t:.1f}s | Tilt: {pitch:.3f} | ZMP: {zmp:.2f} | Force: {total:.1f}N", flush=True)

print("Test complete. Simulation shutting down.")
