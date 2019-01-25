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

MASS = 0.1 # All positions have the same mass
GRAVITY = -9.82

class Deformable:
    # Initialize variables for the deformable
    # Stiffness, forces, etc.
    stiffness = 1.0 #0.5

    def __init__(self, positions = OpenMaya.MPointArray(), velocities = OpenMaya.MPointArray()):
        self.dt = 0.01              # time step for physics computations

        # Attributes to be set by user
        self.elasticity = 0.5       # elasticity coefficient for collisions
        self.friction = 0.5
        self.beta = 0.5

        self.pos_x0 = positions		# rest position (MPoint)
        self.pos_x  = positions 	# current positions (MPoint)
        self.v = convertMayaToNumpyArray(velocities)         # current velocisites (converted from MPoint to vector)

        # Compute variables related to rest positions needed for shape matching
        self.X0 = None
        self.X0_cm = None
        self.q = None
        self.Aqq = None
    #END

    def precomputeDeformVariables(self):
        global MASS

        # Compute things related to rest positions needed for shape matching
        self.X0 = convertMayaToNumpyArray(self.pos_x0)
        self.X0_cm = np.sum(self.X0, axis=0) / self.X0.shape[0]

        # compute relative locations q
        self.q = self.X0 - self.X0_cm

        # Aqq
        self.Aqq = np.zeros((3, 3))

        for i in range(self.q.shape[0]):
            qi = np.array([self.q[i],]) # make sure that we have matrices and not 1D-arrays
            self.Aqq = self.Aqq + MASS * np.dot(qi.T, qi) # Matrix multiplication
        # END FOR

        self.Aqq = np.linalg.inv(self.Aqq)
    #END

    def setCollisionElasticity(self, elasticity):
        self.elasticity = elasticity
    #END

    def setTimeStep(self, timeStep):
        self.dt = timeStep
    #END

    def getPositions(self):
        return self.pos_x
    #END

    def applyForces(self):
        # Apply a force to each position in x
        # access global variables
        global GRAVITY, MASS

        nrPos = self.pos_x.length()

        for i in range(nrPos):
            pos = self.pos_x[i]
            v = self.v[i]

            # compute external forces
            # gravity
            f = GRAVITY * MASS
            F = np.array([0.0, f, 0.0]) # F = [0, -0.982, 0]

            # impulse and friction for collision with ground
            if pos.y <= 0:
                # floor/ground is static
                groundNormal = np.array([0.0, 1.0, 0.0])
                vDiff = v - np.zeros(3) # difference in velocity between the two objects

                # composant in normal direction
                vDiff_par = groundNormal * np.dot(groundNormal, vDiff) # dot product then vector times scalar
                # orthogonal composant
                vDiff_orth = vDiff - vDiff_par 

                # compute impulse and friction
                collisionImpulse = -(self.elasticity + 1) * vDiff_par * MASS
                friction = -self.friction * vDiff_orth * MASS

                F = F + (collisionImpulse + friction) / self.dt
                # move object above ground level
                pos.y = 0.01 
            #END IF

            # update velocities
            self.v[i] = self.v[i] + (F / MASS) * self.dt

            # update positions
            newX = pos.x + self.v[i, 0] * self.dt
            newY = pos.y + self.v[i, 1] * self.dt
            newZ = pos.z + self.v[i, 2] * self.dt

            if newY <= 0:
                newY = 0.01 # move object above ground level
            #END IF

            self.pos_x.set(i, newX, newY, newZ)
        #END FOR
    #END

    def deform(self):
        # Deform object using shape matching
        global MASS

        # Precomputed variables
        X0 = self.X0
        X0_cm = self.X0_cm
        q = self.q
        Aqq = self.Aqq

        # PREPARATIONS
        # convert to array
        X = convertMayaToNumpyArray(self.pos_x)

        # compute optimal translation vector (just the center of mass)
        X_cm = np.sum(X, axis=0) / X.shape[0]   # All have the same mass => No need to include it in the calculation

        # compute relative locations p
        p = X - X_cm

        # compute optimal rotation matrix Apq
        Apq = np.zeros((3, 3))

        for i in range(q.shape[0]):
            # make sure that we have matrices and not 1D-arrays
            pi = np.array([p[i],])
            qi = np.array([q[i],])

            Apq = Apq + MASS * np.dot(pi.T, qi) # Matrix multiplication
        # END FOR

        # Find rotational part in Apq using polar decomposition

        # ALT 0: Polar decomposition
        # ! R = U, rotation (with possible reflection); S = P, scaling/stretching (See wikipedia for explanation)
        # R, S = linalg.polar(Apq)

        # ALT 1: Compute S explicitly
        # S = linalg.sqrtm(np.transpose(Apq)*Apq)
        # R = np.dot(Apq, np.linalg.inv(S))

        # ALT 3: Singular value decomposition (SVD) # NOT THE ONE
        U, D, V = np.linalg.svd(Apq) # V is already transposed
        R = np.dot(V, U)

        # Check determinant to compensate for possible reflections (Not sure if needed)
        if linalg.det(R) < 0:
            R[0, 2] = -R[0, 2]
            R[1, 2] = -R[1, 2]
            R[2, 2] = -R[2, 2]
        # END IF

        # RIGID BODY DEFORMATION
        # compute goal positions
        goalPositions = np.empty( (0, 3) )
        for i in range(X0.shape[0]):
            xDiff = np.array([(X0[i] - X0_cm), ]) # need to be a 2D-array to be multiplied with R

            g = np.dot(R, xDiff.T).T + X_cm
            goalPositions = np.append(goalPositions, g, axis=0)
        # END FOR

        # LINEAR DEFORMATION
        A = np.dot(Apq, Aqq)

        # Ensure that volume is preserved, det(A) = 1
        determinant = linalg.det(A) if linalg.det(A) > 0.001 else 0.001
        A = A / pow(determinant, 1/3)

        # Recompute goal positions
        T = self.beta * A + (1 - self.beta) * R
        goalPositions = np.empty( (0, 3) )
        for i in range(X0.shape[0]):
            xDiff = np.array([(X0[i] - X0_cm), ]) # need to be a 2D-array to be multiplied with R

            g = np.dot(T, xDiff.T).T + X_cm
            goalPositions = np.append(goalPositions, g, axis=0)
        # END FOR

        # INTEGRATION
        # Update current velocities and current position
        for i in range(self.pos_x.length()):
            self.v[i] = self.v[i] + Deformable.stiffness * ((goalPositions[i] - X[i])/self.dt)
            X[i] = X[i] + self.dt * Deformable.stiffness * ((goalPositions[i] - X[i])/self.dt)# self.v[i]

            # convert positions back to MPoint
            self.pos_x.set(i, X[i, 0], X[i, 1], X[i, 2])
        # END FOR
    #END
#END