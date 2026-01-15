import os, math, sys
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Initialize Sensors ---
imu = robot.getDevice('imu')
if imu: imu.enable(timestep)

r_sensor = robot.getDevice('RFootSensor')
l_sensor = robot.getDevice('LFootSensor')
for s in [r_sensor, l_sensor]:
    if s: s.enable(timestep)

r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

print("--- TITAN CORE ONLINE ---", flush=True)

while robot.step(timestep) != -1:
    t = robot.getTime()
    if t > 30.0: break

    # --- PHYSICS WAKE-UP ---
    # In the first 0.5 seconds, we give a tiny nudge to ensure it's not frozen
    nudge = 0.05 if t < 0.5 else 0.0

    # 1. IMU Data
    pitch = imu.getRollPitchYaw()[1] if imu else 0.0
    
    # 2. Simple PID Logic
    error = 0.0 - pitch
    output = (2.5 * error) + nudge
    
    # 3. ZMP/Force Logic
    r_f = abs(r_sensor.getValues()[2]) if r_sensor else 0.0
    l_f = abs(l_sensor.getValues()[2]) if l_sensor else 0.0
    total_f = r_f + l_f
    zmp_y = (r_f - l_f) / total_f if total_f > 1.0 else 0.0

    # 4. Move
    if r_hip: r_hip.setPosition(output)
    if l_hip: l_hip.setPosition(output)

    if int(t * 10) % 20 == 0:
        print(f"Time: {t:.1f}s | Tilt: {pitch:.3f} | ZMP: {zmp_y:.3f} | Load: {total_f:.1f}N", flush=True)

print("Controller session ended.", flush=True)
