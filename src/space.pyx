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

class _SpaceIterator:
    """
    Iterates over the geoms inside a Space (as reported by ODE).
    """
    def __init__(self, space):
        self.space = space
        self.idx = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.idx >= self.space.getNumGeoms():
            raise StopIteration
        else:
            res = self.space.getGeom(self.idx)
            self.idx += 1
            return res

cdef class SpaceBase(GeomObject):
    """SpaceBase class - not initializable by itself

    A space is a non-placeable geom that can contain other geoms.

    It is similar to the rigid body concept of the "world",
    except that it applies to collision instead of dynamics.

    Space objects exist to make collision detection go faster.
    Without spaces, you might generate contacts in your simulation by
    calling dCollide to get contact points for every single pair of geoms.
    For N geoms this is O(N2) tests, which is too computationally expensive
    if your environment has many objects.

    A better approach is to insert the geoms into a space
    and call dSpaceCollide.
    The space will then perform collision culling, which means that it will
    quickly identify which pairs of geoms are potentially intersecting.
    Those pairs will be passed to a callback function, which can in turn call
    dCollide on them. This saves a lot of time that would have been spent in
    useless dCollide tests, because the number of pairs passed to the callback
    function will be a small fraction of every possible object-object pair.

    Spaces can contain other spaces.
    This is useful for dividing a collision environment into
    several hierarchies to further optimize collision detection speed.

    """
    cdef dSpaceID sid

    def __cinit__(self, *a, **kw):
        self.sid = NULL

    def __init__(self, *a, **kw):
        raise NotImplementedError, "The SpaceBase class can't be used directly."

    def __dealloc__(self):
        if self.sid != NULL:
            dSpaceDestroy(self.sid)
            self.sid = NULL
    def _id(self):
        cdef long id
        id = <long>self.sid
        return id

    def __len__(self):
        return self.getNumGeoms()

    def __iter__(self):
        return _SpaceIterator(self)

    def add(self, GeomObject geom):
        """add(geom)

        Add a geom to a space.

        The geom must not have been added to a space before.

        NOTE:
        This function only creates a weakref to the geom object.
        What this means is that geoms can get garbage collected if no other
        (strong) references remain to them.
        Proper management of references is the duty of the client code.

        @param geom: Geom object to add
        @type geom: GeomObject
        """
        dSpaceAdd(self.sid, geom.gid)

    def remove(self, GeomObject geom):
        """remove(geom)

        Remove a geom from a space.

        @param geom: Geom object to remove
        @type geom: GeomObject
        """
        dSpaceRemove(self.sid, geom.gid)

    def query(self, GeomObject geom):
        """query(geom) -> bool

        Return True if the given geom is in the space.

        @param geom: Geom object to check
        @type geom: GeomObject
        """
        return dSpaceQuery(self.sid, geom.gid)

    def getNumGeoms(self):
        """getNumGeoms() -> int

        Return the number of geoms contained within the space.
        """
        return dSpaceGetNumGeoms(self.sid)

    def getGeom(self, int idx):
        """getGeom(idx) -> GeomObject

        Return the geom with the given index contained within the space.

        @param idx: Geom index (0,1,...,getNumGeoms()-1)
        @type idx: int
        """
        # Check the index
        if idx < 0 or idx >= dSpaceGetNumGeoms(self.sid):
            raise IndexError, "geom index out of range"

        cdef dGeomID gid
        gid = dSpaceGetGeom(self.sid, idx)
        if <long>gid not in _geom_c2py_lut:
            raise RuntimeError, "geom id cannot be translated to a Python object"

        return _geom_c2py_lut[<long>gid]

    def setCleanup(self, int mode):
      """setCleanup(int mode)

      Set the clean-up mode of the space.

      If the clean-up mode is 1, then the contained geoms will be destroyed
      when the space is destroyed.
      If the clean-up mode is 0 this does not happen.

      The default clean-up mode for new spaces is 1.

      @param mode: The cleanup mode, 0 or 1
      @type mode: int
      """
      # Check mode
      if not (mode == 0 or mode == 1):
          raise RuntimeError, "Cleanup mode needs to be either 0 or 1."

      dSpaceSetCleanup(self.sid, mode)

    def getCleanup(self):
        """getCleanup() -> int

        Get the clean-up mode of the space.
        """
        return dSpaceGetCleanup(self.sid)

    def setSublevel(self, int sublevel):
        """setSublevel(int sublevel)

        Set the sublevel value for the space.
        Sublevel affects how the space is handled in dSpaceCollide2 when it
        is collided with another space.

        If sublevels of both spaces match, the function iterates geometries of
        both spaces and collides them with each other.

        If sublevel of one space is greater than the sublevel of another one,
        only the geometries of the space with greater sublevel are iterated,
        another space is passed into collision callback as a geometry itself.

        By default all the spaces are assigned zero sublevel.

        NOTE:
        The space sublevel IS NOT automatically updated when one space is
        inserted into another or removed from one.
        It is a client's responsibility to update sublevel value if necessary.

        @param sublevel: The desired sublevel.
        @type sublevel: int
        """
        dSpaceSetSublevel(self.sid, sublevel)

    def getSublevel(self):
        """getSublevel() -> int

        Get the sublevel value for the space.
        """
        return dSpaceGetSublevel(self.sid)

    def collide(self, arg, callback):
        """collide(arg, callback)

        Call a callback function one or more times, for all
        potentially intersecting objects in the space. The callback
        function takes 3 arguments:

        def NearCallback(arg, geom1, geom2):

        The arg parameter is just passed on to the callback function.
        Its meaning is user defined. The geom1 and geom2 arguments are
        the geometry objects that may be near each other. The callback
        function can call the function collide() (not the Space
        method) on geom1 and geom2, perhaps first determining
        whether to collide them at all based on other information.

        @param arg: A user argument that is passed to the callback function
        @param callback: Callback function
        @type callback: callable
        """

        cdef void* data
        cdef object tup
        tup = (callback, arg)
        data = <void*>tup
        dSpaceCollide(self.sid, data, collide_callback)


# Callback function for the dSpaceCollide() call in the Space.collide() method
# The data parameter is a tuple (Python-Callback, Arguments).
# The function calls a Python callback function with 3 arguments:
# def callback(UserArg, Geom1, Geom2)
# Geom1 and Geom2 are instances of GeomXyz classes.
cdef void collide_callback(void* data, dGeomID o1, dGeomID o2):
    cdef object tup
#    cdef Space space
    cdef long id1, id2

#    if (dGeomGetBody(o1)==dGeomGetBody(o2)):
#        return

    tup = <object>data
    callback, arg = tup
    id1 = <long>o1
    id2 = <long>o2
    g1=_geom_c2py_lut[id1]
    g2=_geom_c2py_lut[id2]
    callback(arg,g1,g2)


cdef class SimpleSpace(SpaceBase):
    """Simple space.

    This does not do any collision culling - it simply checks every
    possible pair of geoms for intersection, and reports the pairs
    whose AABBs overlap.

    The time required to do intersection testing for n objects is O(n**2).

    This should not be used for large numbers of objects, but it can be
    the preferred algorithm for a small number of objects.

    This is also useful for debugging potential problems with the
    collision system.
    """
    def __cinit__(self, space = None):
        cdef SpaceBase sp
        cdef dSpaceID parentid

        parentid = NULL
        if space != None:
            sp = space
            parentid = sp.sid

        self.sid = dSimpleSpaceCreate(parentid)

        # Copy the ID
        self.gid = <dGeomID>self.sid

        dSpaceSetCleanup(self.sid, 0)
        _geom_c2py_lut[<long>self.sid]=self

    def __init__(self, space = None):
        pass

cdef class HashSpace(SpaceBase):
    """Multi-resolution hash table space.

    This uses an internal data structure that records how each geom
    overlaps cells in one of several three dimensional grids.

    Each grid has cubical cells of side lengths 2**i, where i is an integer
    that ranges from a minimum to a maximum value.

    The time required to do intersection testing for n objects is O(n)
    (as long as those objects are not clustered together too closely),
    as each object can be quickly paired with the objects around it.
    """
    def __cinit__(self, space = None):
        cdef SpaceBase sp
        cdef dSpaceID parentid

        parentid = NULL
        if space != None:
            sp = space
            parentid = sp.sid

        self.sid = dHashSpaceCreate(parentid)

        # Copy the ID
        self.gid = <dGeomID>self.sid

        dSpaceSetCleanup(self.sid, 0)
        _geom_c2py_lut[<long>self.sid]=self

    def __init__(self, space = None):
        pass

    def setLevels(self, int minlevel, int maxlevel):
        """setLevels(minlevel, maxlevel)

        Sets the size of the smallest and largest cell used in the hash table.
        The actual size will be 2 ** minlevel and 2 ** maxlevel respectively.
        """
        if minlevel > maxlevel:
            raise ValueError, "minlevel ({0}) must be less than or equal to maxlevel ({1})".format(minlevel, maxlevel)

        dHashSpaceSetLevels(self.sid, minlevel, maxlevel)

    def getLevels(self):
        """getLevels() -> (minlevel, maxlevel)

        Gets the size of the smallest and largest cell used in the hash table.
        The actual size is 2 ** minlevel and 2 ** maxlevel respectively.
        """
        cdef int minlevel
        cdef int maxlevel
        dHashSpaceGetLevels(self.sid, &minlevel, &maxlevel)
        return (minlevel, maxlevel)

cdef class QuadTreeSpace(SpaceBase):
    """Quadtree space.

    This uses a pre-allocated hierarchical grid-based AABB tree to quickly
    cull collision checks.
    It's exceptionally quick for large amounts of objects in landscape-shaped
    worlds. The amount of
    memory used is 4 ** depth * 32 bytes.

    Currently getGeom() is not implemented for the quadtree space.
    """

    def __cinit__(self, center, extents, depth, space = None):
        cdef SpaceBase sp
        cdef dSpaceID parentid
        cdef dVector3 c
        cdef dVector3 e

        parentid = NULL
        if space != None:
            sp = space
            parentid = sp.sid

        c[0] = center[0]
        c[1] = center[1]
        c[2] = center[2]
        e[0] = extents[0]
        e[1] = extents[1]
        e[2] = extents[2]
        self.sid = dQuadTreeSpaceCreate(parentid, c, e, depth)

        # Copy the ID
        self.gid = <dGeomID>self.sid

        dSpaceSetCleanup(self.sid, 0)
        _geom_c2py_lut[<long>self.sid]=self

    def __init__(self, center, extents, depth, space = None):
        pass

def Space(space_type = 0):
    """Space factory function.

    Depending on the type argument this function either returns a
    SimpleSpace (space_type = 0) or a HashSpace (space_type = 1).

    This function is provided to remain compatible with previous
    versions of PyODE where there was only one Space class.

     >>> space = Space(space_type=0)   # Create a SimpleSpace
     >>> space = Space(space_type=1)   # Create a HashSpace
    """
    if space_type == 0:
        return SimpleSpace()
    elif space_type == 1:
        return HashSpace()
    else:
        raise ValueError, "Unknown space type ({0})".format(space_type)
