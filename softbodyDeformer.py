#!/usr/bin/env python
# softbodyDeformer.py

# Import dependencies
import sys
import maya.OpenMayaMPx as OpenMayaMPx
import maya.OpenMaya as OpenMaya
import maya.OpenMayaAnim as OpenMayaAnim
import numpy as np

#import deformable  # OBS! Modules are not reloaded unless MAYA is restarted.
from deformable import *

# Plug-in information:
kPluginNodeName = 'softbodyDeformer'     # The name of the node.
kPluginNodeId = OpenMaya.MTypeId( 0x00000002 ) # A unique ID associated to this node type.

# Some global variables were moved from MPxDeformerNode to MPxGeometryFilter. 
# Set some constants to the proper C++ cvars based on the API version.
import maya.cmds as cmds
kApiVersion = cmds.about(apiVersion=True)
if kApiVersion < 201600:
    kInput = OpenMayaMPx.cvar.MPxDeformerNode_input
    kInputGeom = OpenMayaMPx.cvar.MPxDeformerNode_inputGeom
    kOutputGeom = OpenMayaMPx.cvar.MPxDeformerNode_outputGeom
    kEnvelope = OpenMayaMPx.cvar.MPxDeformerNode_envelope
else:
    kInput = OpenMayaMPx.cvar.MPxGeometryFilter_input
    kInputGeom = OpenMayaMPx.cvar.MPxGeometryFilter_inputGeom
    kOutputGeom = OpenMayaMPx.cvar.MPxGeometryFilter_outputGeom
    kEnvelope = OpenMayaMPx.cvar.MPxGeometryFilter_envelope

##########################################################
# Plug-in 
##########################################################
class SoftbodyDeformerNode(OpenMayaMPx.MPxDeformerNode):
    
    # Static variable(s) which will later be replaced by the node's attribute(s).
    time = OpenMaya.MObject()

    def __init__(self):
        ''' Constructor. '''
        # (!) Make sure you call the base class's constructor.
        OpenMayaMPx.MPxDeformerNode.__init__(self)

        self.dObject = Deformable() # The class handling the computation of the deformation
        self.initialised = False 
        self.prevTime = OpenMaya.MTime()
    #END
    
    # pGeometryIterator (MItGeometry)
    def deform(self, pDataBlock, pGeometryIterator, pLocalToWorldMatrix, pGeometryIndex):
        ''' Deform each vertex using the geometry iterator. '''
        
        # Get the values for attributes.
        timeHandle = pDataBlock.inputValue(SoftbodyDeformerNode.time)
        currentTime = timeHandle.asTime()

        frame = int(currentTime.asUnits(OpenMaya.MTime.kFilm))
        
        # Get the input mesh from the datablock using our getDeformerInputGeometry() helper function.     
        inputGeometryObject = self.getDeformerInputGeometry(pDataBlock, pGeometryIndex)     # OBS! Perhaps use this to get the rest positions instead?

        # IF not initialized: Initialize rest shape
        if (frame == 1 or not self.initialised): 
            print 'Initialising deformable object...'

            # Save original positions of all points into another object
            # Obtain the list of positions for each vertex in the mesh.
            restPositions = OpenMaya.MPointArray()

            pGeometryIterator.allPositions(restPositions)

            # Convert to world coordinates
            for i in range(restPositions.length()):
                restPositions.set(restPositions[i] * pLocalToWorldMatrix, i)
            #END FOR

            # Initialise velocity for the points
            initVelX = pDataBlock.inputValue(SoftbodyDeformerNode.initialVelocityX).asDouble()
            initVelY = pDataBlock.inputValue(SoftbodyDeformerNode.initialVelocityY).asDouble()
            initVelZ = pDataBlock.inputValue(SoftbodyDeformerNode.initialVelocityZ).asDouble()

            restVelocities = OpenMaya.MPointArray(restPositions.length(), OpenMaya.MPoint(initVelX, initVelY, initVelZ))

            # Set mass of particles
            mass = pDataBlock.inputValue(SoftbodyDeformerNode.particleMass).asDouble()

            # Initialise deformable object
            self.dObject = Deformable(restPositions, restVelocities, mass)
            self.dObject.precomputeDeformVariables()
            
            self.prevTime = currentTime
            self.initialised = True

        # ELSE: Update shape
        elif (self.prevTime != currentTime):
            # Set variables used for physics simulation
            self.dObject.setCollisionElasticity(pDataBlock.inputValue(SoftbodyDeformerNode.collisionElasticity).asDouble())
            self.dObject.setCollisionFriction(pDataBlock.inputValue(SoftbodyDeformerNode.collisionFriction).asDouble())
            self.dObject.setBeta(pDataBlock.inputValue(SoftbodyDeformerNode.beta).asDouble())
            self.dObject.setStiffness(pDataBlock.inputValue(SoftbodyDeformerNode.stiffness).asDouble())
            self.dObject.setJiggleness(pDataBlock.inputValue(SoftbodyDeformerNode.jiggleness).asDouble())

            # Set timestep
            diffTime = currentTime - self.prevTime   

            nrUpdates = int(diffTime.value())
            nrUpdatesPerTimestep = 3
            dt = (1/24.0)/nrUpdatesPerTimestep * (-1 if nrUpdates < 0 else 1) # 24 fps
            self.dObject.setTimeStep(dt)

            # Update previous time (to use for next time step)
            self.prevTime = currentTime

            for i in range(0, abs(nrUpdates*nrUpdatesPerTimestep)):
                # Apply forces
                self.dObject.applyForces()

                # Update positions using forces and shape matching
                self.dObject.deform()
            #END FOR

            # Update output positions
            newPositions = self.dObject.getPositions() 

            # Convert to model coordinates
            for i in range(newPositions.length()):
                newPositions.set(newPositions[i] * pLocalToWorldMatrix.inverse(), i)
            #END FOR

            # Update the object's positions 
            pGeometryIterator.setAllPositions(newPositions)
        #END IF
    #END
    
    def getDeformerInputGeometry(self, pDataBlock, pGeometryIndex):
        '''
        Obtain a reference to the input mesh. This mesh will be used to compute our bounding box, and we will also require its normals.
        
        We use MDataBlock.outputArrayValue() to avoid having to recompute the mesh and propagate this recomputation throughout the 
        Dependency Graph.
        
        OpenMayaMPx.cvar.MPxDeformerNode_input and OpenMayaMPx.cvar.MPxDeformerNode_inputGeom (for pre Maya 2016) and 
        OpenMayaMPx.cvar.MPxGeometryFilter_input and OpenMayaMPx.cvar.MPxGeometryFilter_inputGeom (Maya 2016) are SWIG-generated 
        variables which respectively contain references to the deformer's 'input' attribute and 'inputGeom' attribute.   
        '''
        inputAttribute = OpenMayaMPx.cvar.MPxGeometryFilter_input
        inputGeometryAttribute = OpenMayaMPx.cvar.MPxGeometryFilter_inputGeom
        
        inputHandle = pDataBlock.outputArrayValue( inputAttribute )
        inputHandle.jumpToElement( pGeometryIndex )
        inputGeometryObject = inputHandle.outputValue().child( inputGeometryAttribute ).asMesh()
        
        return inputGeometryObject
    #END
#END

##########################################################
# Plug-in initialization.
##########################################################
def nodeCreator():
    ''' Creates an instance of our node class and delivers it to Maya as a pointer. '''
    return OpenMayaMPx.asMPxPtr( SoftbodyDeformerNode() )
#END

def nodeInitializer():
    ''' Defines the input and output attributes as static variables in our plug-in class. '''
    # The following MFnNumericAttribute function set will allow us to create our attributes.
    numericAttributeFn = OpenMaya.MFnNumericAttribute()
    unitAttributeFn = OpenMaya.MFnUnitAttribute()

    #==================================
    # INPUT NODE ATTRIBUTE(S)
    #==================================

    SoftbodyDeformerNode.time = unitAttributeFn.create('time', 't', OpenMaya.MFnUnitAttribute.kTime, 0.0)
    unitAttributeFn.setDefault(OpenMayaAnim.MAnimControl.currentTime())
    unitAttributeFn.setStorable( True )
    unitAttributeFn.setWritable( True )
    unitAttributeFn.setKeyable( True )
    unitAttributeFn.setReadable( False )
    unitAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.time)

    # TODO: comment
    SoftbodyDeformerNode.particleMass = numericAttributeFn.create('particleMass', 'mass', OpenMaya.MFnNumericData.kDouble, 0.1)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(10.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.particleMass)

    # Collision elasticity
    SoftbodyDeformerNode.collisionElasticity = numericAttributeFn.create('collisionElasticity', 'ce', OpenMaya.MFnNumericData.kDouble, 0.5)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(1.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.collisionElasticity)

    # TODO: comment
    SoftbodyDeformerNode.collisionFriction = numericAttributeFn.create('collisionFriction', 'cf', OpenMaya.MFnNumericData.kDouble, 0.5)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(1.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.collisionFriction)

    # TODO: comment
    SoftbodyDeformerNode.beta = numericAttributeFn.create('beta', 'beta', OpenMaya.MFnNumericData.kDouble, 0.5)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(1.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.beta)
    
    # TODO: Comment
    SoftbodyDeformerNode.stiffness = numericAttributeFn.create('stiffness', 'stiff', OpenMaya.MFnNumericData.kDouble, 0.5)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(1.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.stiffness)

    # TODO: Comment
    SoftbodyDeformerNode.jiggleness = numericAttributeFn.create('jiggleness', 'jiggle', OpenMaya.MFnNumericData.kDouble, 0.5)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(1.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.jiggleness)

    # TODO: Comment
    SoftbodyDeformerNode.initialVelocityX = numericAttributeFn.create('initialVelocityX', 'initVX', OpenMaya.MFnNumericData.kDouble, 0.0)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(10.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.initialVelocityX)

    SoftbodyDeformerNode.initialVelocityY = numericAttributeFn.create('initialVelocityY', 'initVY', OpenMaya.MFnNumericData.kDouble, 0.0)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(10.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.initialVelocityY)

    SoftbodyDeformerNode.initialVelocityZ = numericAttributeFn.create('initialVelocityZ', 'initVZ', OpenMaya.MFnNumericData.kDouble, 0.0)
    numericAttributeFn.setMin(0.0)
    numericAttributeFn.setMax(10.0)
    numericAttributeFn.setChannelBox( True )
    SoftbodyDeformerNode.addAttribute(SoftbodyDeformerNode.initialVelocityZ)
    
    ''' The input geometry node attribute is already declared in OpenMayaMPx.cvar.MPxGeometryFilter_inputGeom '''

    #==================================
    # OUTPUT NODE ATTRIBUTE(S)
    #==================================

    ''' The output geometry node attribute is already declared in OpenMayaMPx.cvar.MPxGeometryFilter_outputGeom '''

    #==================================
    # NODE ATTRIBUTE DEPENDENCIES
    #==================================
    # If any of the inputs change, the output mesh will be recomputed.

    print dir(OpenMayaMPx.cvar)

    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.time, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.particleMass, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.collisionElasticity, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.collisionFriction, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.beta, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.stiffness, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.jiggleness, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.initialVelocityX, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.initialVelocityY, kOutputGeom)
    SoftbodyDeformerNode.attributeAffects(SoftbodyDeformerNode.initialVelocityZ, kOutputGeom)
#END

def initializePlugin( mobject ):
    ''' Initialize the plug-in '''
    mplugin = OpenMayaMPx.MFnPlugin( mobject )

    try:
        mplugin.registerNode( kPluginNodeName, kPluginNodeId, nodeCreator,
                              nodeInitializer, OpenMayaMPx.MPxNode.kDeformerNode )
    except:
        sys.stderr.write( 'Failed to register node: ' + kPluginNodeName )
        raise
#END

def uninitializePlugin( mobject ):
    ''' Uninitializes the plug-in '''
    mplugin = OpenMayaMPx.MFnPlugin( mobject )
    try:
        mplugin.deregisterNode( kPluginNodeId )
    except:
        sys.stderr.write( 'Failed to deregister node: ' + kPluginNodeName )
        raise
#END

##########################################################
# Sample usage.
##########################################################
''' 
# Copy the following lines and run them in Maya's Python Script Editor:

import maya.cmds as cmds
cmds.loadPlugin( 'X:\TNCG13-SoftbodyDeformer\softbodyDeformer.py' )
cmds.polyCube()
cmds.move(0, 10, 0)
cmds.deformer( type='softbodyDeformer' )
cmds.connectAttr( 'time1.outTime', 'softbodyDeformer1.time' )
'''

'''
# OBS! To avoid Maya crash when reloading plugin:
# Delete any created objects connected with the deformer and 
# run the following MEL command: 

file -new
'''

'''
# To run from MEL instead of Python
loadPlugin("X:/TNCG13-SoftbodyDeformer/softbodyDeformer.py");

// Create a plane
polyPlane -n myPlane -sx 1 -sy 1 -h 10 -w 10;
//select -r myPlane ;
move -r 0 0.0 0;

polySphere -n mySphere -r 1 -sx 20 -sy 20 -ax 0 1 0 -cuv 2 -ch 1;

move -r 0 2.0 0;

select -r mySphere;
CreatePassiveRigidBody;
Gravity;

select -r mySphere;
deformer -type softbodyDeformer;

connectAttr -f time1.outTime softbodyDeformer1.time;

// Connect the dynamics attributes to the deformer node
// Gravity Direction
//connectAttr -f gravityField1.direction softbodyDeformer.GravityDirection;
// Gravity Magnitude
//connectAttr -f gravityField1.magnitude softbodyDeformer.GravityMagnitude;
// Time
//connectAttr -f time1.outTime softbodyDeformer.CurrentTime;
'''	