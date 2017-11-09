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

# This file is included by _trimesh_switch.pyx if the variable
# TRIMESH_SUPPORT was set to True in the setup script.

# GeomTriMesh
cdef class GeomTriMesh(GeomObject):
    """TriMesh object.

    To construct the trimesh geom you need a TriMeshData object that
    stores the actual mesh. This object has to be passed as first
    argument to the constructor.

    Constructor::
    
      GeomTriMesh(data, space=None)    
    """

    # Keep a reference to the data
    cdef TriMeshData data

    def __cinit__(self, TriMeshData data not None, space=None):
        cdef SpaceBase sp
        cdef dSpaceID sid

        self.data = data

        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateTriMesh(sid, data.tmdid, NULL, NULL, NULL)

        _geom_c2py_lut[<long>self.gid] = self


    def __init__(self, TriMeshData data not None, space=None):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id

    def clearTCCache(self):
        """clearTCCache()

        Clears the internal temporal coherence caches.
        """
        dGeomTriMeshClearTCCache(self.gid)

    def getTriangle(self, int idx):
        """getTriangle(idx) -> (v0, v1, v2)

        @param idx: Triangle index
        @type idx: int
        """

        cdef dVector3 v0, v1, v2
        cdef dVector3* vp0
        cdef dVector3* vp1
        cdef dVector3* vp2

        vp0 = <dVector3*>v0
        vp1 = <dVector3*>v1
        vp2 = <dVector3*>v2

        dGeomTriMeshGetTriangle(self.gid, idx, vp0, vp1, vp2)
        return ((v0[0],v0[1],v0[2]), (v1[0],v1[1],v1[2]), (v2[0],v2[1],v2[2]))
        
    def getTriangleCount(self):
        """getTriangleCount() -> n

        Returns the number of triangles in the TriMesh."""

        return dGeomTriMeshGetTriangleCount(self.gid)

