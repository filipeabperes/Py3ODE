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

cdef class TriMeshData:
    """This class stores the mesh data.
    """

    cdef dTriMeshDataID tmdid
    cdef dReal* vertex_buffer
    cdef int* face_buffer

    def __cinit__(self):
        self.tmdid = dGeomTriMeshDataCreate()
        self.vertex_buffer = NULL
        self.face_buffer = NULL

    def __dealloc__(self):
        if self.tmdid!=NULL:
            dGeomTriMeshDataDestroy(self.tmdid)
        if self.vertex_buffer!=NULL:
            free(self.vertex_buffer)
        if self.face_buffer!=NULL:
            free(self.face_buffer)
    
    def build(self, verts, faces):
        """build(verts, faces)

        @param verts: Vertices
        @type verts: Sequence of 3-sequences of floats
        @param faces: Face definitions (three indices per face)
        @type faces: Sequence of 3-sequences of ints
        """
        cdef int numverts
        cdef int numfaces
        cdef dReal* vp
        cdef int* fp
        cdef int a,b,c
        
        numverts = len(verts)
        numfaces = len(faces)
        # Allocate the vertex and face buffer
        self.vertex_buffer = <dReal*>malloc(numverts*4*sizeof(dReal))
        self.face_buffer = <int*>malloc(numfaces*3*sizeof(int))

        # Fill the vertex buffer
        vp = self.vertex_buffer
        for v in verts:
            vp[0] = v[0]
            vp[1] = v[1]
            vp[2] = v[2]
            vp[3] = 0
            vp = vp+4

        # Fill the face buffer
        fp = self.face_buffer
        for f in faces:
            a = f[0]
            b = f[1]
            c = f[2]
            if a<0 or b<0 or c<0 or a>=numverts or b>=numverts or c>=numverts:
                raise ValueError, "Vertex index out of range"
            fp[0] = a
            fp[1] = b
            fp[2] = c
            fp = fp+3

        # Pass the data to ODE
        dGeomTriMeshDataBuildSimple(self.tmdid, self.vertex_buffer, numverts, self.face_buffer, numfaces*3)