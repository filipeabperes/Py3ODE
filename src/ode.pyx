######################################################################
# Python Open Dynamics Engine Wrapper
# Copyright (C) 2004 PyODE developers (see file AUTHORS)
# All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of EITHER:
#   (1) The GNU Lesser General Public License as published by the Free
#       Software Foundation; either version 2.1 of the License, or (at
#       your option) any later version. The text of the GNU Lesser
#       General Public License is included with this library in the
#       file LICENSE.
#   (2) The BSD-style license that is included with this library in
#       the file LICENSE-BSD.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
# LICENSE and LICENSE-BSD for more details. 
######################################################################
# Open Dynamics Engine
# Copyright (c) 2001-2003, Russell L. Smith.
# All rights reserved. 
####################################################################

include "declarations.pyx"

# The World should keep a reference to joints/bodies, so that they won't
# be deleted.

# Excplicitly assign the module doc string to __doc__
# (otherwise it won't show up which is probably a "bug" in Pyrex (v0.9.2.1))
__doc__ = """Python Open Dynamics Engine (ODE) wrapper.

This module contains classes and functions that wrap the functionality
of the Open Dynamics Engine (ODE) which can be found at 
http://opende.sourceforge.net.

There are the following classes and functions:

 - World
 - Body
 - JointGroup
 - Contact
 - Space
 - Mass

Joint classes:

 - BallJoint
 - HingeJoint
 - Hinge2Joint
 - SliderJoint
 - UniversalJoint
 - FixedJoint
 - ContactJoint
 - AMotor
 - LMotor
 - Plane2DJoint

Geom classes:

 - GeomSphere
 - GeomBox
 - GeomPlane
 - GeomCapsule
 - GeomCylinder
 - GeomRay
 - GeomTransform
 - GeomTriMesh / TriMeshData

Functions:

 - CloseODE()
 - collide()

"""

############################# Constants ###############################

paramLoStop        = 0
paramHiStop        = 1
paramVel           = 2
paramFMax          = 3
paramFudgeFactor   = 4
paramBounce        = 5
paramCFM           = 6
paramStopERP       = 7
paramStopCFM       = 8
paramSuspensionERP = 9
paramSuspensionCFM = 10

ParamLoStop        = 0
ParamHiStop        = 1
ParamVel           = 2
ParamFMax          = 3
ParamFudgeFactor   = 4
ParamBounce        = 5
ParamCFM           = 6
ParamStopERP       = 7
ParamStopCFM       = 8
ParamSuspensionERP = 9
ParamSuspensionCFM = 10

ParamLoStop2        = 256+0
ParamHiStop2        = 256+1
ParamVel2           = 256+2
ParamFMax2          = 256+3
ParamFudgeFactor2   = 256+4
ParamBounce2        = 256+5
ParamCFM2           = 256+6
ParamStopERP2       = 256+7
ParamStopCFM2       = 256+8
ParamSuspensionERP2 = 256+9
ParamSuspensionCFM2 = 256+10

ParamLoStop3        = 512+0
ParamHiStop3        = 512+1
ParamVel3           = 512+2
ParamFMax3          = 512+3
ParamFudgeFactor3   = 512+4
ParamBounce3        = 512+5
ParamCFM3           = 512+6
ParamStopERP3       = 512+7
ParamStopCFM3       = 512+8
ParamSuspensionERP3 = 512+9
ParamSuspensionCFM3 = 512+10

ParamGroup = 256

ContactMu2	= 0x001
ContactFDir1	= 0x002
ContactBounce	= 0x004
ContactSoftERP	= 0x008
ContactSoftCFM	= 0x010
ContactMotion1	= 0x020
ContactMotion2	= 0x040
ContactSlip1	= 0x080
ContactSlip2	= 0x100

ContactApprox0 = 0x0000
ContactApprox1_1	= 0x1000
ContactApprox1_2	= 0x2000
ContactApprox1	= 0x3000

AMotorUser = dAMotorUser
AMotorEuler = dAMotorEuler

Infinity = dInfinity

######################################################################

# Lookup table for geom objects: C ptr -> Python object

## This causes some kind of weird bug! Need to fix this.
import weakref
_geom_c2py_lut = weakref.WeakValueDictionary()

# Mass 
include "mass.pyx"

# Contact
include "contact.pyx"

# World
include "world.pyx"

# Body
include "body.pyx"

# Joint classes
include "joints.pyx"

# Geom base
include "geomobject.pyx"

# Space
include "space.pyx"

# Geom classes
include "geoms.pyx"

# Include the generated trimesh switch file that either includes the real
# trimesh wrapper (trimesh.pyx/trimeshdata.pyx) or a dummy wrapper
# (trimesh_dummy.pyx) if trimesh support is not available/desired.
include "_trimesh_switch.pyx"

include "heightfielddata.pyx"
include "heightfield.pyx"
    
def collide(geom1, geom2):
    """collide(geom1, geom2) -> contacts

    Generate contact information for two objects.

    Given two geometry objects that potentially touch (geom1 and geom2),
    generate contact information for them. Internally, this just calls
    the correct class-specific collision functions for geom1 and geom2.

    [flags specifies how contacts should be generated if the objects
    touch. Currently the lower 16 bits of flags specifies the maximum
    number of contact points to generate. If this number is zero, this
    function just pretends that it is one - in other words you can not
    ask for zero contacts. All other bits in flags must be zero. In
    the future the other bits may be used to select other contact
    generation strategies.]

    If the objects touch, this returns a list of Contact objects,
    otherwise it returns an empty list.

    @param geom1: First Geom
    @type geom1: GeomObject
    @param geom2: Second Geom
    @type geom2: GeomObject
    @returns: Returns a list of Contact objects.
    """
    
    cdef dContactGeom c[150]
    cdef long id1
    cdef long id2
    cdef int i, n
    cdef Contact cont

    id1 = geom1._id()
    id2 = geom2._id()

    n = dCollide(<dGeomID>id1, <dGeomID>id2, 150, c, sizeof(dContactGeom))
    res = []
    i=0
    while i<n:
        cont = Contact()
        cont._contact.geom = c[i]
        res.append(cont)
        i=i+1

    return res

def collide2(geom1, geom2, arg, callback):
    """collide2(geom1, geom2, arg, callback)
    
    Calls the callback for all potentially intersecting pairs that contain
    one geom from geom1 and one geom from geom2.

    @param geom1: First Geom
    @type geom1: GeomObject
    @param geom2: Second Geom
    @type geom2: GeomObject
    @param arg: A user argument that is passed to the callback function
    @param callback: Callback function
    @type callback: callable    
    """
    cdef void* data
    cdef object tup
    cdef long id1
    cdef long id2

    id1 = geom1._id()
    id2 = geom2._id()
    
    tup = (callback, arg)
    data = <void*>tup
    # collide_callback is defined in space.pyx
    dSpaceCollide2(<dGeomID>id1, <dGeomID>id2, data, collide_callback)


def areConnected(Body body1, Body body2):
    """areConnected(body1, body2) -> bool

    Return True if the two bodies are connected together by a joint,
    otherwise return False.

    @param body1: First body
    @type body1: Body
    @param body2: Second body
    @type body2: Body
    @returns: True if the bodies are connected
    """

    if (body1 is environment):
        return False
    if (body2 is environment):
        return False

    return bool(dAreConnected(<dBodyID> body1.bid, <dBodyID> body2.bid))

def CloseODE():
    """CloseODE()

    Deallocate some extra memory used by ODE that can not be deallocated
    using the normal destroy functions.
    """
    dCloseODE()

def InitODE():
    '''InitODE()

    Initialize some ODE internals. This will be called for you when you 
    "import ode", but you should call this again if you CloseODE().'''
    dInitODE()

######################################################################

#environment = Body(None)
environment = None
InitODE()
