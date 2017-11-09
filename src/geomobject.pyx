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

# Each geom object has to insert itself into the global dictionary
# _geom_c2py_lut (key:address - value:Python object).
# This lookup table is used in the near callback to translate the C
# pointers into corresponding Python wrapper objects.
#
# Additionally, each geom object must have a method _id() that returns
# the ODE geom id. This is used during collision detection.
# 
# ##########################
# # Obsolete:
# #
# # Each geom object has to register itself at its space as the
# # space keeps a dictionary that's used as lookup table to translate
# # C pointers into Python objects (this is used in the near callback).


# Geom base class
cdef class GeomObject:
    """This is the abstract base class for all geom objects.
    """
    
    # The id of the geom object as returned by dCreateXxxx()
    cdef dGeomID gid
    # The space in which the geom was placed (or None). This reference
    # is kept so that the space won't be destroyed while there are still
    # geoms around that might use it. 
    cdef object space
    # The body that the geom was attached to (or None).
    cdef object body

    # A dictionary with user defined attributes
    cdef object attribs

    cdef object __weakref__

    def __cinit__(self, *a, **kw):
        self.gid = NULL
        self.space = None
        self.body = None
        self.attribs = {}

    def __init__(self, *a, **kw):
        raise NotImplementedError, "The GeomObject base class can't be used directly."

    def __dealloc__(self):
        if self.gid!=NULL:
            dGeomDestroy(self.gid)
            self.gid = NULL

    def __getattr__(self, name):
        if name in self.attribs:
            return self.attribs[name]
        else:
            raise AttributeError, "geom has no attribute '%s'."%name

    def __setattr__(self, name, val):
        self.attribs[name]=val

    def __delattr__(self, name):
        if name in self.attribs:
            del self.attribs[name]
        else:
            raise AttributeError, "geom has no attribute '%s'."%name

    def _id(self):
        """_id() -> int

        Return the internal id of the geom (dGeomID) as returned by
        the dCreateXyz() functions.

        This method has to be overwritten in derived methods.        
        """
        raise NotImplementedError, "Bug: The _id() method is not implemented."

    def placeable(self):
        """placeable() -> bool

        Returns True if the geom object is a placeable geom.

        This method has to be overwritten in derived methods.
        """
        return False

    def setBody(self, Body body):
        """setBody(body)

        Set the body associated with a placeable geom.

        @param body: The Body object or None.
        @type body: Body
        """

        if not self.placeable():
            raise ValueError, "Non-placeable geoms cannot have a body associated to them."
        
        if body==None:
            dGeomSetBody(self.gid, NULL)
        else:
            dGeomSetBody(self.gid, body.bid)
        self.body = body

    def getBody(self):
        """getBody() -> Body

        Get the body associated with this geom.
        """
        if not self.placeable():
            return environment
        
        return self.body

    def setPosition(self, pos):
        """setPosition(pos)

        Set the position of the geom. If the geom is attached to a body,
        the body's position will also be changed.

        @param pos: Position
        @type pos: 3-sequence of floats
        """
        if not self.placeable():
            raise ValueError, "Cannot set a position on non-placeable geoms."
        dGeomSetPosition(self.gid, pos[0], pos[1], pos[2])

    def getPosition(self):
        """getPosition() -> 3-tuple

        Get the current position of the geom. If the geom is attached to
        a body the returned value is the body's position.
        """
        if not self.placeable():
            raise ValueError, "Non-placeable geoms do not have a position."

        cdef dReal* p
        p = <dReal*>dGeomGetPosition(self.gid)
        return (p[0],p[1],p[2])

    def setRotation(self, R):
        """setRotation(R)

        Set the orientation of the geom. If the geom is attached to a body,
        the body's orientation will also be changed.

        @param R: Rotation matrix
        @type R: 9-sequence of floats
        """
        if not self.placeable():
            raise ValueError, "Cannot set a rotation on non-placeable geoms."

        cdef dMatrix3 m
        m[0] = R[0]
        m[1] = R[1]
        m[2] = R[2]
        m[3] = 0
        m[4] = R[3]
        m[5] = R[4]
        m[6] = R[5]
        m[7] = 0
        m[8] = R[6]
        m[9] = R[7]
        m[10] = R[8]
        m[11] = 0
        dGeomSetRotation(self.gid, m)

    def getRotation(self):
        """getRotation() -> 9-tuple

        Get the current orientation of the geom. If the geom is attached to
        a body the returned value is the body's orientation.
        """
        if not self.placeable():
            raise ValueError, "Non-placeable geoms do not have a rotation."

        cdef dReal* m
        m = <dReal*>dGeomGetRotation(self.gid)
        return [m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10]]

    def getQuaternion(self):
        """getQuaternion() -> (w,x,y,z)

        Get the current orientation of the geom. If the geom is attached to
        a body the returned value is the body's orientation.
        """
        if not self.placeable():
            raise ValueError, "Non-placeable geoms do not have an orientation."

        cdef dQuaternion q
        dGeomGetQuaternion(self.gid, q)
        return (q[0],q[1],q[2],q[3])

    def setQuaternion(self, q):
        """setQuaternion(q)

        Set the orientation of the geom. If the geom is attached to a body,
        the body's orientation will also be changed.

        @param q: Quaternion (w,x,y,z)
        @type q: 4-sequence of floats
        """
        if not self.placeable():
            raise ValueError, "Cannot set a quaternion on non-placeable geoms."

        cdef dQuaternion cq
        cq[0] = q[0]
        cq[1] = q[1]
        cq[2] = q[2]
        cq[3] = q[3]
        dGeomSetQuaternion(self.gid, cq)

    def getAABB(self):
        """getAABB() -> 6-tuple

        Return an axis aligned bounding box that surrounds the geom.
        The return value is a 6-tuple (minx, maxx, miny, maxy, minz, maxz).
        """
        cdef dReal aabb[6]
        
        dGeomGetAABB(self.gid, aabb)
        return (aabb[0], aabb[1], aabb[2], aabb[3], aabb[4], aabb[5])

    def isSpace(self):
        """isSpace() -> bool

        Return 1 if the given geom is a space, or 0 if not."""
        return dGeomIsSpace(self.gid)

    def getSpace(self):
        """getSpace() -> Space

        Return the space that the given geometry is contained in,
        or return None if it is not contained in any space."""        
        return self.space

    def setCollideBits(self, bits):
        """setCollideBits(bits)

        Set the "collide" bitfields for this geom.

        @param bits: Collide bit field
        @type bits: int/long
        """
        dGeomSetCollideBits(self.gid, long(bits))
        
    def setCategoryBits(self, bits):
        """setCategoryBits(bits)

        Set the "category" bitfields for this geom.

        @param bits: Category bit field
        @type bits: int/long
        """
        dGeomSetCategoryBits(self.gid, long(bits))

    def getCollideBits(self):
        """getCollideBits() -> long

        Return the "collide" bitfields for this geom.
        """
        return dGeomGetCollideBits(self.gid)

    def getCategoryBits(self):
        """getCategoryBits() -> long

        Return the "category" bitfields for this geom.
        """
        return dGeomGetCategoryBits(self.gid)
    
    def enable(self):
        """enable()

        Enable the geom."""
        dGeomEnable(self.gid)

    def disable(self):
        """disable()

        Disable the geom."""
        dGeomDisable(self.gid)

    def isEnabled(self):
        """isEnabled() -> bool

        Return True if the geom is enabled."""
        return dGeomIsEnabled(self.gid)



