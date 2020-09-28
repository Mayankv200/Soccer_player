"""Supervisor controller"""

from controller import Supervisor
from controller import Emitter
import struct

# Supervisor setup

supervisor = Supervisor()
TIMESTEP = int(supervisor.getBasicTimeStep())
COMM_CHANNEL = -1

# Supervisor interpret world
soccerball = supervisor.getFromDef("BALL")
trans_field = soccerball.getField("translation")
ball_radius = 0.113# soccerball.getField("radius")
INITIAL_TRANS = [0, ball_radius, 0]

# Emitter setup

emitter = supervisor.getEmitter('emitter')
emitter.setChannel(COMM_CHANNEL)
timeLastMessage = -1

while supervisor.step(TIMESTEP) != -1:

    values = trans_field.getSFVec3f()

    # Emit ball position
    if supervisor.getTime() > timeLastMessage + 1:
        message = struct.pack("ddd", values[0], values[1], values[2])
        emitter.send(message)
        timeLastMessage = int(supervisor.getTime())
    
    # determine out of bounds64
    if values[0] > 5:
        trans_field.setSFVec3f([5, ball_radius, values[2]])
        soccerball.resetPhysics()
    elif values[0] < -5:
        trans_field.setSFVec3f([-5, ball_radius, values[2]])
        soccerball.resetPhysics()
    elif values[2] > 3:
        trans_field.setSFVec3f([values[0], ball_radius, 3])
        soccerball.resetPhysics()
    elif values[2] < -3:
        trans_field.setSFVec3f([values[0], ball_radius, -3])
        soccerball.resetPhysics()
        
    if (values[2] > 0.75) or (values[2] < -0.75):
        if (values[0] > 4.5):
            trans_field.setSFVec3f([4.5, ball_radius, values[2]])
            soccerball.resetPhysics()
        elif (values[0] < -4.5):
            trans_field.setSFVec3f([-4.5, ball_radius, values[2]])
            soccerball.resetPhysics()
    # determine in goal
    if ((values[2] < 0.75) and (values[2] >-0.75)):
        if ((values[0] > 4.5) and (values[0] < 5)) or ((values[0] < -4.5) and (values[0] > -5)):
            trans_field.setSFVec3f(INITIAL_TRANS)
            soccerball.resetPhysics()