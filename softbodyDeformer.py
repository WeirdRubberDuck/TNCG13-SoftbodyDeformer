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
        if (not self.initialised): # Initialize on first frame

            print 'Initialising deformable object...'

            # Save original positions of all points into another object
            # Obtain the list of positions for each vertex in the mesh.
            restPositions = OpenMaya.MPointArray()
            meshFn = OpenMaya.MFnMesh( inputGeometryObject )
            meshFn.getPoints( restPositions, OpenMaya.MSpace.kTransform ) # OBS: This can be done using pGeomertyIterator instead
        
            # Init velocity for the points
            restVelocities = OpenMaya.MPointArray(restPositions.length(), OpenMaya.MPoint()) # value = 0.0

            self.dObject = Deformable(restPositions, restVelocities)
            self.prevTime = currentTime
            self.initialised = True

            print 'prevTime: ' + str(self.prevTime.value())

        # ELSE: Update shape
        elif (self.prevTime != currentTime):
            
            #print 'Updating object positions...'

            # Set timestep
            diffTime = currentTime - self.prevTime
            
            nrUpdates = int(diffTime.value())
            dt = (1/24.0) * (-1 if nrUpdates < 0 else 1) # 24 fps
            self.dObject.setTimeStep(dt)

            #print 'dt: ' + str(dt)
            #print 'prevTime: ' + str(self.prevTime.value())
            #print 'currentTime: ' + str(currentTime.value())
            #print 'nrUpdates: ' + str(nrUpdates)

            # update previous time (to use for next time step)
            self.prevTime = currentTime

            for i in range(0, abs(nrUpdates)):
                # Apply forces
                self.dObject.applyForces()
                self.dObject.deform()

                newPositions = self.dObject.getPositions()  # OBS! There is a class called MPointArray, perhaps use that? For rest positions as well.
                
                # Update the object's positions 
                pGeometryIterator.setAllPositions(newPositions)
            
                # Update positions using forces and shape matching

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
cmds.loadPlugin( 'H:\TNCG13-SoftbodyDeformer\softbodyDeformer.py' )
cmds.polyCube()
cmds.deformer( type='softbodyDeformer' )

# OBS! To avoid Maya crash when reloading plugin:
# Delete any created objects connected with the deformer and 
# run the following MEL command: 

 file -new

'''	