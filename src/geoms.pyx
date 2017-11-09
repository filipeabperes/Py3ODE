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

# The list of geom classes supported in ODE, and whether or not they
# are placeable, is at
# http://opende.sourceforge.net/wiki/index.php/Manual_%28Collision_Detection%29

# GeomSphere
cdef class GeomSphere(GeomObject):
    """Sphere geometry.

    This class represents a sphere centered at the origin.

    Constructor::
    
      GeomSphere(space=None, radius=1.0)
    """

    def __cinit__(self, space=None, radius=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid

        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateSphere(sid, radius)
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self


    def __init__(self, space=None, radius=1.0):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setRadius(self, radius):
        """setRadius(radius)

        Set the radius of the sphere.

        @param radius: New radius
        @type radius: float
        """
        dGeomSphereSetRadius(self.gid, radius)

    def getRadius(self):
        """getRadius() -> float

        Return the radius of the sphere.
        """
        return dGeomSphereGetRadius(self.gid)

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the sphere. Points inside
        the geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomSpherePointDepth(self.gid, p[0], p[1], p[2])

                
# GeomBox
cdef class GeomBox(GeomObject):
    """Box geometry.

    This class represents a box centered at the origin.

    Constructor::
    
      GeomBox(space=None, lengths=(1.0, 1.0, 1.0))
    """

    def __cinit__(self, space=None, lengths=(1.0, 1.0, 1.0)):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateBox(sid, lengths[0],lengths[1],lengths[2])
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self

    def __init__(self, space=None, lengths=(1.0, 1.0, 1.0)):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setLengths(self, lengths):
        dGeomBoxSetLengths(self.gid, lengths[0], lengths[1], lengths[2])

    def getLengths(self):
        cdef dVector3 res
        dGeomBoxGetLengths(self.gid, res)
        return (res[0], res[1], res[2])

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the box. Points inside the
        geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomBoxPointDepth(self.gid, p[0], p[1], p[2])


# GeomPlane
cdef class GeomPlane(GeomObject):
    """Plane geometry.

    This class represents an infinite plane. The plane equation is:
    n.x*x + n.y*y + n.z*z = dist

    This object can't be attached to a body.
    If you call getBody() on this object it always returns ode.environment.

    Constructor::
    
      GeomPlane(space=None, normal=(0,0,1), dist=0)

    """

    def __cinit__(self, space=None, normal=(0,0,1), dist=0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreatePlane(sid, normal[0], normal[1], normal[2], dist)
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self


    def __init__(self, space=None, normal=(0,0,1), dist=0):
        self.space = space


    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setParams(self, normal, dist):
        dGeomPlaneSetParams(self.gid, normal[0], normal[1], normal[2], dist)

    def getParams(self):
        cdef dVector4 res
        dGeomPlaneGetParams(self.gid, res)
        return ((res[0], res[1], res[2]), res[3])

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the plane. Points inside the
        geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomPlanePointDepth(self.gid, p[0], p[1], p[2])


# GeomCapsule
cdef class GeomCapsule(GeomObject):
    """Capped cylinder geometry.

    This class represents a capped cylinder aligned along the local Z axis
    and centered at the origin.

    Constructor::
    
      GeomCapsule(space=None, radius=0.5, length=1.0)

    The length parameter does not include the caps.
    """

    def __cinit__(self, space=None, radius=0.5, length=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateCapsule(sid, radius, length)
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self

    def __init__(self, space=None, radius=0.5, length=1.0):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setParams(self, radius, length):
        dGeomCapsuleSetParams(self.gid, radius, length)

    def getParams(self):
        cdef dReal radius, length
        dGeomCapsuleGetParams(self.gid, &radius, &length)
        return (radius, length)

    def pointDepth(self, p):
        """pointDepth(p) -> float

        Return the depth of the point p in the cylinder. Points inside the
        geom will have positive depth, points outside it will have
        negative depth, and points on the surface will have zero
        depth.

        @param p: Point
        @type p: 3-sequence of floats
        """
        return dGeomCapsulePointDepth(self.gid, p[0], p[1], p[2])

GeomCCylinder = GeomCapsule # backwards compatibility


# GeomCylinder
cdef class GeomCylinder(GeomObject):
    """Plain cylinder geometry.

    This class represents an uncapped cylinder aligned along the local Z axis
    and centered at the origin.

    Constructor::
    
      GeomCylinder(space=None, radius=0.5, length=1.0)
    """

    def __cinit__(self, space=None, radius=0.5, length=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateCylinder(sid, radius, length)
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self

    def __init__(self, space=None, radius=0.5, length=1.0):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setParams(self, radius, length):
        dGeomCylinderSetParams(self.gid, radius, length)

    def getParams(self):
        cdef dReal radius, length
        dGeomCylinderGetParams(self.gid, &radius, &length)
        return (radius, length)

    ## dGeomCylinderPointDepth not implemented upstream in ODE 0.7


# GeomRay
cdef class GeomRay(GeomObject):
    """Ray object.

    A ray is different from all the other geom classes in that it does
    not represent a solid object. It is an infinitely thin line that
    starts from the geom's position and extends in the direction of
    the geom's local Z-axis.

    Constructor::
    
      GeomRay(space=None, rlen=1.0)
    
    """

    def __cinit__(self, space=None, rlen=1.0):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateRay(sid, rlen)
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self


    def __init__(self, space=None, rlen=1.0):
        self.space = space
        self.body = None

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def placeable(self):
        return True

    def setLength(self, rlen):
        '''setLength(rlen)

        Set length of the ray.

        @param rlen: length of the ray
        @type rlen: float'''
        dGeomRaySetLength(self.gid, rlen)

    def getLength(self):
        '''getLength() -> length

        Get the length of the ray.

        @returns: length of the ray (float)'''
        return dGeomRayGetLength(self.gid)

    def set(self, p, u):
        '''set(p, u)

        Set the position and rotation of a ray.
        
        @param p: position
        @type p: 3-sequence of floats
        @param u: rotation
        @type u: 3-sequence of floats'''
        dGeomRaySet(self.gid, p[0],p[1],p[2], u[0],u[1],u[2])

    def get(self):
        '''get() -> ((p[0], p[1], p[2]), (u[0], u[1], u[2]))

        Return the position and rotation as a pair of
        tuples.

        @returns: position and rotation'''
        cdef dVector3 start
        cdef dVector3 dir
        dGeomRayGet(self.gid, start, dir)
        return ((start[0],start[1],start[2]), (dir[0],dir[1],dir[2]))


# GeomTransform
cdef class GeomTransform(GeomObject):
    """GeomTransform.

    A geometry transform "T" is a geom that encapsulates another geom
    "E", allowing E to be positioned and rotated arbitrarily with
    respect to its point of reference.

    Constructor::
    
      GeomTransform(space=None)    
    """

    cdef object geom

    def __cinit__(self, space=None):
        cdef SpaceBase sp
        cdef dSpaceID sid
        
        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateGeomTransform(sid)
        # Set cleanup mode to 0 as a contained geom will be deleted
        # by its Python wrapper class
        dGeomTransformSetCleanup(self.gid, 0)
#        if space!=None:
#            space._addgeom(self)

        _geom_c2py_lut[<long>self.gid]=self

    def __init__(self, space=None):
        self.space = space
        self.body = None
        self.geom = None

        self.attribs={}

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def setGeom(self, GeomObject geom not None):
        """setGeom(geom)

        Set the geom that the geometry transform encapsulates.
        A ValueError exception is thrown if a) the geom is not placeable,
        b) the geom was already inserted into a space or c) the geom is
        already associated with a body.

        @param geom: Geom object to encapsulate
        @type geom: GeomObject
        """
        cdef long id

        if not geom.placeable():
            raise ValueError, "Only placeable geoms can be encapsulated by a GeomTransform"
        if dGeomGetSpace(geom.gid)!=<dSpaceID>0:
            raise ValueError, "The encapsulated geom was already inserted into a space."
        if dGeomGetBody(geom.gid)!=<dBodyID>0:
            raise ValueError, "The encapsulated geom is already associated with a body."
        
        id = geom._id()
        dGeomTransformSetGeom(self.gid, <dGeomID>id)
        self.geom = geom

    def getGeom(self):
        """getGeom() -> GeomObject

        Get the geom that the geometry transform encapsulates.
        """
        return self.geom

    def setInfo(self, int mode):
        """setInfo(mode)

        Set the "information" mode of the geometry transform.

        With mode 0, when a transform object is collided with another
        object, the geom field of the ContactGeom structure is set to the
        geom that is encapsulated by the transform object.

        With mode 1, the geom field of the ContactGeom structure is set
        to the transform object itself.

        @param mode: Information mode (0 or 1)
        @type mode: int
        """
        if mode<0 or mode>1:
            raise ValueError, "Invalid information mode (%d). Must be either 0 or 1."%mode
        dGeomTransformSetInfo(self.gid, mode)

    def getInfo(self):
        """getInfo() -> int

        Get the "information" mode of the geometry transform (0 or 1).

        With mode 0, when a transform object is collided with another
        object, the geom field of the ContactGeom structure is set to the
        geom that is encapsulated by the transform object.

        With mode 1, the geom field of the ContactGeom structure is set
        to the transform object itself.
        """
        return dGeomTransformGetInfo(self.gid)


