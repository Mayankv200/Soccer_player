"""Receiver controller."""

# Import modules
from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Receiver
import struct

# Create robot instance
robot = Robot()

# Declare constansts
TIMESTEP = int(robot.getBasicTimeStep())
COMM_CHANNEL = -1
ROBOT_SPEED = 5.0

# Sensor setup

ds = []
dsNames = ['ds0', 'ds1']
for name in dsNames:
    ds.append(robot.getDistanceSensor(name))
for i in range(2):
    ds[i].enable(TIMESTEP)
isTooClose = False

# Motor setup

wheels = []
wheelsNames = ['left wheel motor', 'right wheel motor']
for name in wheelsNames:
    wheels.append(robot.getMotor(name))
for i in range(2):
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# Receiver setup

receiver = robot.getReceiver('receiver')
receiver.enable(1)
receiver.setChannel(COMM_CHANNEL)

# Main loop:

while robot.step(TIMESTEP) != -1:

    # Just receiver stuff...
    
    if receiver.getQueueLength() > 0:
        newMessage = receiver.getData()
        message = struct.unpack("ddd", newMessage)
        print(f"{message[0]} {message[1]} {message[2]}")
        receiver.nextPacket()
        
    leftSpeed = ROBOT_SPEED
    rightSpeed = ROBOT_SPEED

    # Read sensors
    for i in range(2):
        if ds[i].getValue() > 500.0:
            leftSpeed += ROBOT_SPEED
            rightSpeed -= ROBOT_SPEED
            if ds[i].getValue() > 900:
                leftSpeed += ROBOT_SPEED
                rightSpeed -= ROBOT_SPEED
                if ds[i].getValue() > 950:
                    isTooClose = True
    if isTooClose == True:
        leftSpeed = 0
        rightSpeed = ROBOT_SPEED * -3.0
        isTooClose = False
    
    # Act on sensor data
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)