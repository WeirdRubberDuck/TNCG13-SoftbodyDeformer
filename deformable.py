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
        self.qTilde = None
        self.AqqTilde = None
    #END

    ''' Precomputation of variables based on rest poisitions. Needed for deformation '''
    def precomputeDeformVariables(self):
        global MASS

        # Compute things related to rest positions needed for shape matching
        self.X0 = convertMayaToNumpyArray(self.pos_x0)
        self.X0_cm = np.sum(self.X0, axis=0) / self.X0.shape[0]

        # Compute relative locations q
        self.q = self.X0 - self.X0_cm

        # Compute Aqq, AqqTilde and qTilde
        self.Aqq = np.zeros((3, 3)) 
        self.AqqTilde = np.zeros((9, 9))

        self.qTilde = np.zeros((self.q.shape[0], 9))
        for i in range(self.q.shape[0]):
            # Compute Aqq
            qi = np.array([self.q[i],]) # make sure that we have matrices and not 1D-arrays
            self.Aqq = self.Aqq + MASS * np.dot(qi.T, qi) # matrix multiplication

            # Compute AqqTilde and updated relative positions qTilde (used for quadratic deformation)
            qx = self.q[i][0]
            qy = self.q[i][1]
            qz = self.q[i][2]
            self.qTilde[i] = np.array([qx, qy, qz, qx*qx, qy*qy, qz*qz, qx*qy, qy*qz, qz*qx])

            qiTilde = np.array([self.qTilde[i],])
            self.AqqTilde = self.AqqTilde + MASS * np.dot(qiTilde.T, qiTilde) # matrix multiplication
        # END FOR

        # Inverse deformation matrices
        self.Aqq = np.linalg.inv(self.Aqq)
        self.AqqTilde = np.linalg.inv(self.AqqTilde)   
    #END

    def setCollisionElasticity(self, elasticity):
        self.elasticity = elasticity
    #END

    def setCollisionFriction(self, friction):
        self.friction = friction
    #END

    def setBeta(self, beta):
        self.beta = beta
    #END

    def setTimeStep(self, timeStep):
        self.dt = timeStep
    #END

    def getPositions(self):
        return self.pos_x
    #END

    ''' 
    Compute goal positions based on relative rest positions q, 
    transformation matrix T and updated center of mass X_cm 
    '''
    def computeGoalPositions(self, q, T, X_cm):
        goalPositions = np.empty( (0, 3) )
        for i in range(q.shape[0]):
            xDiff = np.array([q[i], ]) # 2D matrix
            g = np.dot(T, xDiff.T).T + X_cm
            goalPositions = np.append(goalPositions, g, axis=0)
        # END FOR
        return goalPositions
    #END

    ''' Apply a force to each position in x '''
    def applyForces(self):
        # Access global variables
        global GRAVITY, MASS
        nrPos = self.pos_x.length()

        # TODO: Make sure that forces works correctly. (DET BALLAR UR!!)

        for i in range(nrPos):
            pos = self.pos_x[i]
            v = self.v[i]

            # Compute external forces
            # gravity
            f = GRAVITY * MASS
            F = np.array([0.0, f, 0.0]) # F = [0, -0.982, 0]

            # Impulse and friction for collision with ground
            if pos.y <= 0:
                # Floor/ground is static
                groundNormal = np.array([0.0, 1.0, 0.0])
                vDiff = v - np.zeros(3) # difference in velocity between the two objects

                # Composant in normal direction
                vDiff_par = groundNormal * np.dot(groundNormal, vDiff) # dot product then vector times scalar
                # Orthogonal composant
                vDiff_orth = vDiff - vDiff_par 

                # Compute impulse and friction
                collisionImpulse = -(self.elasticity + 1) * vDiff_par * MASS
                friction = -self.friction * vDiff_orth * MASS

                F = F + (collisionImpulse + friction) / self.dt
                # Move object above ground level
                pos.y = 0.01 
            #END IF

            # Update velocities
            self.v[i] = self.v[i] + (F / MASS) * self.dt

            # Update positions
            newX = pos.x + self.v[i, 0] * self.dt
            newY = pos.y + self.v[i, 1] * self.dt
            newZ = pos.z + self.v[i, 2] * self.dt

            # Move object above ground level
            if newY <= 0: newY = 0.01

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
        qTilde = self.qTilde
        AqqTilde = self.AqqTilde

        # PREPARATIONS
        # Convert to array
        X = convertMayaToNumpyArray(self.pos_x)

        # Compute optimal translation vector (just the center of mass)
        X_cm = np.sum(X, axis=0) / X.shape[0]   # all have the same mass => No need to include it in the calculation

        # Compute relative locations p
        p = X - X_cm

        # Compute optimal rotation matrix Apq
        Apq = np.zeros((3, 3))
        for i in range(q.shape[0]):
            # Make sure that we have matrices and not 1D-arrays
            pi = np.array([p[i],])
            qi = np.array([q[i],])
            Apq = Apq + MASS * np.dot(pi.T, qi) # matrix multiplication
        # END FOR

        # Find rotational part in Apq using singular value decomposition (SVD)
        U, D, V = np.linalg.svd(Apq) # V is already transposed
        R = np.dot(V, U)

        # Check determinant to compensate for possible reflections (Not sure if needed)
        if linalg.det(R) < 0:
            R[0, 2] = -R[0, 2]
            R[1, 2] = -R[1, 2]
            R[2, 2] = -R[2, 2]
        # END IF

        # RIGID BODY DEFORMATION
        goalPositions = self.computeGoalPositions(q, R, X_cm)
        
        # LINEAR DEFORMATION
        A = np.dot(Apq, Aqq)

        # Ensure that volume is preserved, det(A) = 1
        determinant = linalg.det(A) if linalg.det(A) > 0.001 else 0.001
        A = A / pow(determinant, 1/3)

        # Compute goal positions
        T = self.beta * A + (1 - self.beta) * R
        goalPositions = self.computeGoalPositions(q, T, X_cm)

        # QUADRATIC DEFORMATION
        ApqTilde = np.zeros((3, 9))
        for i in range(q.shape[0]):
            # make sure that we have matrices and not 1D-arrays
            pi = np.array([p[i],])
            qi = np.array([qTilde[i],])
            ApqTilde = ApqTilde + MASS * np.dot(pi.T, qi) # Matrix multiplication
        # END FOR

        ATilde = np.dot(ApqTilde, AqqTilde)

        # TODO: Preserve volume for linear part of ATilde

        # Compute RTilde = [R 0 0]
        zero = np.zeros((3, 6))
        RTilde = np.concatenate((R, zero), axis=1)

        # Compute goal positions
        T = self.beta * ATilde + (1 - self.beta) * RTilde
        goalPositions = self.computeGoalPositions(qTilde, T, X_cm)

        # INTEGRATION
        # Update current velocities and current position
        for i in range(self.pos_x.length()):
            self.v[i] = self.v[i] + Deformable.stiffness * ((goalPositions[i] - X[i])/self.dt)
            X[i] = X[i] + self.dt * Deformable.stiffness * ((goalPositions[i] - X[i])/self.dt)# self.v[i]

            # Convert positions back to MPoint
            self.pos_x.set(i, X[i, 0], X[i, 1], X[i, 2])
        # END FOR
    #END
#END