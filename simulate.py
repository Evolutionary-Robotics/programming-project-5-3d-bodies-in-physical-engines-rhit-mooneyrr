import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 
from generate import tentacle_positions, tentacle_segments

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#p.loadSDF("box.sdf")
robotId = p.loadURDF("Octo_Body.urdf")

duration = 5000

ps.Prepare_To_Simulate(robotId)

x = np.linspace(0,10*np.pi, duration)
y = np.sin(x)*np.pi/4

for i in range(duration):
    
    for t in range(len(tentacle_positions)):
        for seg in range(tentacle_segments):
            joint_name = f"Tentacle_{t+1}_Joint_{seg+1}"
            ps.Set_Motor_For_Joint(
                bodyIndex=robotId,
                jointName=bytes(joint_name, 'utf-8'),
                controlMode=p.POSITION_CONTROL,
                targetPosition=y[i],
                maxForce=500
            )

    p.stepSimulation()
    time.sleep(1/500)

p.disconnect()