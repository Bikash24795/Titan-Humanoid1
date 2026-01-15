import os
from controller import Robot

# The brain connecting to the body
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get devices
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

print("--- CONTROLLER CONNECTED TO TITAN SUCCESSFUL ---")

while robot.step(timestep) != -1:
    import math
    val = math.sin(robot.getTime()) * 0.5
    r_hip.setPosition(val)
    l_hip.setPosition(-val)
    
    # ADD THIS LINE TO SEE PROGRESS IN COLAB:
    if int(robot.getTime()) % 2 == 0: 
        print(f"Robot Time: {robot.getTime():.2f} - Hips moving to {val:.2f}")
