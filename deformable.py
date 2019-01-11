# deformable.py

'''
Useful shit:
Ctrl-K Ctrl-C = comments every line
Ctrl-K Ctrl-U = un-comments every line
'''


import maya.OpenMaya as OpenMaya
import numpy as np
import scipy as sp
from scipy import linalg

def convertMayaToNumpyArray(inPositions):
    # initialize numpy array to use for computations
    np.set_printoptions(threshold=np.nan)
    outPositions = np.empty( (0, 3) )
    for i in range(inPositions.length()):
        numpyPos = np.array( [inPositions[i].x, inPositions[i].y, inPositions[i].z] )
        outPositions = np.append(outPositions, [numpyPos], axis=0) 
    #END FOR
    return outPositions
#END

class Deformable:
    # Initialize variables for the deformable
    # Stiffness, mass, etc.
    mass = 1.0 # All positions have the same mass
    gravity = -9.82
    stiffness = 1.0 #0.5
    externalForces = 1.0
    
    def __init__(self, positions = OpenMaya.MPointArray(), velocities = OpenMaya.MPointArray()):
        self.dt = 0.01              # time step for physics computations
        self.pos_x0 = positions		# rest position (MPoint)
        self.pos_x  = positions 	# current positions (MPoint)  
        self.v = convertMayaToNumpyArray(velocities)         # current velocisites (converted from MPoint to vector)
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
        print "Applying forces..."
        #for i in range(self.pos_x.length()):
        #pos = self.pos_x[i]
        #print('x: ' + str(pos.x) + ' y: ' + str(pos.y) + ' z: ' + str(pos.z)) 
        #pos.y = pos.y - 2.0 * self.dt
        #END FOR
    #END

    def deform(self):
        # Deform object using shape matching
        
        # STEP 0: PREPARATIONS
        # convert to array
        X = convertMayaToNumpyArray(self.pos_x)
        X0 = convertMayaToNumpyArray(self.pos_x0)

        # STEP 1: COMPUTE GOAL POSITIONS
        # compute optimal translation vector (just the center of mass)
        X_cm = np.sum(X, axis=0) / X.shape[0]   # All have the same mass => No need to include it in the calculation
        X0_cm = np.sum(X0, axis=0) / X0.shape[0]

        # compute relative locations q and p
        q = np.subtract(X0, X0_cm)
        p = np.subtract(X, X_cm)
        # print "p:"
        # print(p)
        # print "q:"
        # print(q)

        # compute optimal rotation matrix
        # Apq
        Apq = np.zeros((3, 3))

        for i in range(q.shape[0]):
            # make sure that we have matrices and not 1D-arrays
            pi = np.array([p[i],])
            qi_T = np.transpose(np.array([q[i],]))

            # compute Apq
            Apq = Apq + Deformable.mass * pi * qi_T
        # END FOR

        # print "Apq:"
        # print(Apq)

        # find rotational part in Apq using polar decomposition

        # ALT 0: Polar decomposition
        # R = U, rotation (with possible reflection); S = P, scaling/stretching (See wikipedia for explanation)
        R, S = linalg.polar(Apq)

        # ALT 1: Compute S explicitly
        # S = linalg.sqrtm(np.transpose(Apq)*Apq)
        # R = Apq * np.linalg.inv(S)

        # ALT 3: Singular value decomposition (SVD)
        # U, D, V = np.linalg.svd(Apq) # V is already transposed
        # R = V*np.transpose(U)

        # print "R:"
        # print(R)
        
        # CORRECT SO FAR (we think so)

        # Check determinant to compensate for possible reflections (Not sure if needed)
        # if np.linalg.det(R) < 0:
        #     R[0, 2] = -R[0, 2]
        #     R[1, 2] = -R[1, 2]
        #     R[2, 2] = -R[2, 2]
        # # END IF

        # compute goal positions
        goalPositions = np.empty( (0, 3) )
        for i in range(X0.shape[0]):
            xDiff = np.transpose(np.array([(X0[i] - X0_cm), ]))

            g = np.transpose(np.dot(R, xDiff)) + X_cm
            goalPositions = np.append(goalPositions, g, axis=0) 
        # END FOR

        # print "goalPositions:"
        # print(goalPositions)

        # STEP 2: INTEGRATION
        # update current velocities and current position
        for i in range(self.pos_x.length()):
            # TODO: Make sure multiplications work correctly!!
            self.v[i] = self.v[i] + Deformable.stiffness * ((goalPositions[i] - X[i])/self.dt) # TODO: add external forces
            X[i] = X[i] + self.dt * self.v[i] 

            # convert positions back to MPoint
            self.pos_x.set(i, X[i, 0], X[i, 1], X[i, 2])
        # END FOR
    #END
#END