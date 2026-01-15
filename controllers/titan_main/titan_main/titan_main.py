import os, math, sys
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Initialize Sensors ---
imu = robot.getDevice('imu')
imu.enable(timestep)

r_sensor = robot.getDevice('RFootSensor')
l_sensor = robot.getDevice('LFootSensor')
r_sensor.enable(timestep)
l_sensor.enable(timestep)

# --- Initialize Motors ---
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

# --- PID Variables ---
target_pitch = 0.0
kp, ki, kd = 2.5, 0.1, 0.5
last_error, integral = 0, 0

print("--- TITAN HUMAN-LOGIC ONLINE ---", flush=True)

while robot.step(timestep) != -1:
    # 1. IMU DATA (Inner Ear)
    pitch = imu.getRollPitchYaw()[1]
    
    # 2. PID BALANCE
    error = target_pitch - pitch
    integral += error
    derivative = error - last_error
    output = (kp * error) + (ki * integral) + (kd * derivative)
    
    # 3. ZMP CALCULATION (Foot Pressure)
    r_force = r_sensor.getValues()[2] # Vertical force on right foot
    l_force = l_sensor.getValues()[2] # Vertical force on left foot
    total_force = r_force + l_force
    
    # Simple ZMP: If weight is 100% on one side, ZMP is at that foot
    if total_force > 0:
        zmp_y = (r_force - l_force) / total_force
    else:
        zmp_y = 0

    # Apply PID output to Hip Motors to catch the fall
    r_hip.setPosition(output)
    l_hip.setPosition(output)
    
    last_error = error

    if int(robot.getTime() * 10) % 20 == 0:
        print(f"Time: {robot.getTime():.2f}s | Pitch: {pitch:.2f} | ZMP_Y: {zmp_y:.2f}", flush=True)
