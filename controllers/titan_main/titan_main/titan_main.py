import os, sys, math
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Safe Initialization ---
def get_device(name, type):
    dev = robot.getDevice(name)
    if dev:
        if type == "sensor": dev.enable(timestep)
        return dev
    print(f"MISSING DEVICE: {name}", flush=True)
    return None

imu = get_device('imu', 'sensor')
r_sensor = get_device('RFootSensor', 'sensor')
l_sensor = get_device('LFootSensor', 'sensor')
r_hip = get_device('RHip', 'motor')
l_hip = get_device('LHip', 'motor')

print("--- TITAN STABILITY SYSTEM ONLINE ---", flush=True)

while robot.step(timestep) != -1:
    t = robot.getTime()
    if t > 30.0: break

    # 1. Sense
    pitch = imu.getRollPitchYaw()[1] if imu else 0.0
    
    # 2. Calculate Balance (Simplified PID)
    # If pitch is positive (falling forward), move hips to compensate
    error = 0.0 - pitch
    output = 2.5 * error 
    
    # 3. ZMP (Pressure)
    r_f = abs(r_sensor.getValues()[2]) if r_sensor else 0.0
    l_f = abs(l_sensor.getValues()[2]) if l_sensor else 0.0
    total_f = r_f + l_f
    zmp_y = (r_f - l_f) / total_f if total_f > 1.0 else 0.0

    # 4. Actuate
    if r_hip: r_hip.setPosition(output)
    if l_hip: l_hip.setPosition(-output) # Opposite direction for balance

    if int(t * 10) % 20 == 0:
        print(f"T: {t:.1f}s | Pitch: {pitch:.3f} | ZMP: {zmp_y:.3f} | Force: {total_f:.1f}", flush=True)
