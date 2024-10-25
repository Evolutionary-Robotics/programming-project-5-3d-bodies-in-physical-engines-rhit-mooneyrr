import pyrosim.pyrosim as ps
import pybullet as p
import pybullet_data
import numpy as np
import time
import matplotlib.pyplot as plt 

# Global parameters
core_radius = 0.5
tentacle_length = 1.0
segment_length = 1.0
tentacle_segments = 1  # int(tentacle_length / segment_length)
tentacle_radius = 0.1

tentacle_positions = [
    [core_radius, core_radius, 0.0],    # Front-right corner
    [-core_radius, core_radius, 0.0],   # Front-left corner
    [core_radius, -core_radius, 0.0],   # Back-right corner
    [-core_radius, -core_radius, 0.0],  # Back-left corner
]

def Create_World():
    ps.Start_SDF("Octo_World.sdf")
    ps.Send_Cube(name="Floor", pos=[0,0,0], size=[5,5,0.1])
    ps.End()

def Create_Tentacle(base_pos, tentacle_id):
    joint_name = f"Tentacle_{tentacle_id}_Joint"
    segment_name = f"Tentacle_{tentacle_id}_Segment_0"

    ps.Send_Joint(
        name=joint_name,
        parent= "Octopus_Core",
        child=segment_name,
        type="revolute",
        position=[base_pos[0], base_pos[1], base_pos[2]]
    )

    ps.Send_Cube(name=segment_name, pos=[base_pos[0], base_pos[1], base_pos[2] - segment_length/2], size=[tentacle_radius, tentacle_radius, segment_length])

def Create_Octopus():
    ps.Start_URDF("Octo_Body.urdf")

    ps.Send_Cube(name="Octopus_Core", pos=[0,0,0.5], size=[core_radius*2, core_radius*2, core_radius*2])

    for i, pos in enumerate(tentacle_positions):
        Create_Tentacle(base_pos=pos, tentacle_id=i+1)
    
    ps.End()

Create_World()
Create_Octopus()

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("Octo_Body.urdf")

duration = 5000

ps.Prepare_To_Simulate(robotId)


frequency = 0.2 
amplitude = np.pi / 6 
phase_offsets = [0, np.pi / 2, np.pi, 3 * np.pi / 2]  

time_steps = []
driving_signals = [[] for _ in range(len(tentacle_positions))]

# Main simulation loop
for i in range(duration):

    for t in range(len(tentacle_positions)):
        joint_name = f"Tentacle_{t+1}_Joint"
        target_angle = amplitude * np.sin(frequency * i + phase_offsets[t])
        
        driving_signals[t].append(target_angle)
        
        ps.Set_Motor_For_Joint(
            bodyIndex=robotId,
            jointName=bytes(joint_name, 'utf-8'),
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            maxForce=500
        )

    # Step simulation
    p.stepSimulation()
    time.sleep(1/500)

p.disconnect()

# Plot the driving signals
plt.figure(figsize=(10, 6))
for t in range(len(tentacle_positions)):
    plt.plot(time_steps, driving_signals[t], label=f'Tentacle {t+1} Signal')

plt.title("Driving Signals for Each Tentacle Joint")
plt.xlabel("Time Step")
plt.ylabel("Target Angle (radians)")
plt.legend()
plt.grid(True)
plt.show()