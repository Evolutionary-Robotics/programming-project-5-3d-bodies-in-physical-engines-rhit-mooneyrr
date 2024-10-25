import pyrosim.pyrosim as ps

# Global parameters
l = 1 # length
w = 1 # width 
h = 1 # height 

x = 0
y = 0
z = 0.5

def Create_World():
    ps.Start_SDF("box.sdf")
    for i in range(10):
        ps.Send_Cube(name="Box",pos=[x,y,z],size=[l,w,h])
        z += h
        l = 0.9 * l
        w = 0.9 * w
        h = 0.9 * h
    ps.End()

def Create_Robot():
    ps.Start_URDF("body.urdf")
    ps.Send_Cube(name="Foot",pos=[x,y,z],size=[l,w,h]) # Parent
    ps.Send_Joint(name="Foot_Torso", parent="Foot", child="Torso", type="revolute", position = [0.5,0,1.0])
    ps.Send_Cube(name="Torso",pos=[0.5,0.0,0.5],size=[l,w,h]) # Child
    ps.End()

Create_Robot()
