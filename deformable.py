# deformable.py

import maya.OpenMaya as OpenMaya

class Deformable:
    # Initialize variables for the deformable
    # Stiffness, mass, etc.
    mass = 1.0
    gravity = -9.82
    
    def __init__(self, positions = OpenMaya.MPointArray(), velocities = OpenMaya.MPointArray()):
        self.dt = 0.01              # time step for physics computations
        self.pos_x0 = positions		# rest position (MPoint)
        self.pos_x  = positions 	# current positions (MPoint)  
        self.v = velocities         # current velocisites (MPoint)
    #END

    def setTimeStep(self, timeStep):
        self.dt = timeStep
    #END

    def getPositions(self):
        return self.pos_x
    #END
    
    def applyForces(self):    
        # print 'Number vertices in pos_x = ' + str(self.pos_x.length())
        # Apply a force to each position in x
        for i in range(self.pos_x.length()):
            pos = self.pos_x[i]
            #print('x: ' + str(pos.x) + ' y: ' + str(pos.y) + ' z: ' + str(pos.z)) 
            pos.y = pos.y - 2.0 * self.dt
        #END FOR
    #END

    #def deform(self):
        # Deform object using shape matching
        
        # STEP 1: COMPUTE GOAL POSITIONS

        # compute optimal translation vector

        # compute optimal rotation matrix
        # Apq
        # Singular value decomposition?

        # STEP 2: INTEGRATION

    #END
#END