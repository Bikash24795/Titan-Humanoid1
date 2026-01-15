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

# --- Initialize Motors ---
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

# --- PID Balance Constants ---
target_pitch = 0.0
kp, ki, kd = 2.0, 0.05, 0.4
last_error, integral = 0, 0

print("--- TITAN STABILITY SYSTEM: RUNNING 30s TEST ---", flush=True)

while robot.step(timestep) != -1:
    t = robot.getTime()
    
    # 1. STOP LOGIC (Ensures video saves correctly)
    if t > 30.0:
        print("30 Seconds reached. Saving video and exiting...", flush=True)
        break

    # 2. IMU & PID Calculation
    pitch = imu.getRollPitchYaw()[1] if imu else 0.0
    error = target_pitch - pitch
    integral += error
    derivative = error - last_error
    output = (kp * error) + (ki * integral) + (kd * derivative)
    
    # 3. ZMP Logic (Vertical Force)
    r_f = r_sensor.getValues()[2] if r_sensor else 0.0
    l_f = l_sensor.getValues()[2] if l_sensor else 0.0
    total_f = abs(r_f) + abs(l_f)
    zmp_y = (r_f - l_f) / total_f if total_f > 0.1 else 0.0

    # 4. Motor Actuation
    if r_hip and l_hip:
        r_hip.setPosition(output)
        l_hip.setPosition(output)
    
    last_error = error

    # Log data every 2 seconds to keep Colab clean
    if int(t * 10) % 20 == 0:
        print(f"Time: {t:.1f}s | Tilt: {pitch:.2f} | ZMP: {zmp_y:.2f}", flush=True)

# Final command to ensure Webots knows we are done
print("Controller Finished.", flush=True)
