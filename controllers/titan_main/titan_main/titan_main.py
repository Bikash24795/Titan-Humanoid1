import os
from controller import Robot

# This tells Python to talk to the local Webots instance
os.environ['WEBOTS_ROBOT_NAME'] = 'Titan'

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize Motors
r_hip = robot.getDevice('RHip')
l_hip = robot.getDevice('LHip')

while robot.step(timestep) != -1:
    # Make the robot "dance" slightly to test the motors
    import math
    val = math.sin(robot.getTime()) * 0.5
    r_hip.setPosition(val)
    l_hip.setPosition(-val)