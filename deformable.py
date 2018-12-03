# deformable.py

import maya.OpenMaya as OpenMaya

class Deformable:
    # Initialize variables for the deformable
    # Stiffness, mass, etc.
    
    mass = 1.0
    gravity = -9.82

    # Time step for physics computations
    dt = 0.0
    
    def __init__(self, positions, magicNr):
        self.pos_x0 = positions		# rest position (MPoint)
        self.pos_x  = positions 	# current positions (MPoint)  
    #END

    def setTimeStep(self, timeStep):
        dt = timeStep
    #END

    def getPositions(self):
        return self.pos_x
    #END
    
    def applyForces(self):    
        # Apply a force to each position in x
        for pos in self.pos_x:
            print('x: ' + str(pos.x) + ' y: ' + str(pos.y) + ' z: ' + str(pos.z)) 
        #END FOR
    #END
#END