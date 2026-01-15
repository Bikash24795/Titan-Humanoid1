import os, math, sys
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Initialize Sensors with Safety ---
imu = robot.getDevice('imu')
if imu: imu.enable(timestep)

r_sensor = robot.getDevice('RFootSensor')
l_sensor = robot.getDevice('LFootSensor')

for s in [r_sensor, l_sensor]:
    if s: s.enable(timestep)
    else: print("CRITICAL: Foot Sensor not found! Check PROTO names.", flush=True)

# --- Initialize Motors ---
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

# --- PID Variables ---
target_pitch = 0.0
kp, ki, kd = 2.0, 0.05, 0.4
last_error, integral = 0, 0

print("--- TITAN STABILITY SYSTEM STARTING ---", flush=True)

while robot.step(timestep) != -1:
    # 1. IMU Pitch Data
    pitch = imu.getRollPitchYaw()[1] if imu else 0.0
    
    # 2. PID Math
    error = target_pitch - pitch
    integral += error
    derivative = error - last_error
    output = (kp * error) + (ki * integral) + (kd * derivative)
    
    # 3. ZMP Calculation
    # We get the vertical force (Index 2 in force-3d)
    r_f = r_sensor.getValues()[2] if r_sensor else 0.0
    l_f = l_sensor.getValues()[2] if l_sensor else 0.0
    
    total_f = abs(r_f) + abs(l_f)
    zmp_y = (r_f - l_f) / total_f if total_f > 0.1 else 0.0

    # 4. Actuation
    if r_hip and l_hip:
        r_hip.setPosition(output)
        l_hip.setPosition(output)
    
    last_error = error

    # Log data every 1 second
    if int(robot.getTime() * 10) % 10 == 0:
        print(f"Time: {robot.getTime():.1f}s | Pitch: {pitch:.2f} | ZMP: {zmp_y:.2f}", flush=True)
