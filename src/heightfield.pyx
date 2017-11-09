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

cdef class GeomHeightfield(GeomObject):
    """Heightfield object.

    To construct the heightfield geom, you need a HeightfieldData object that
    stores the heightfield data. This object has to be passed as the first
    argument to the constructor.

    Constructor::
        GeomHeightfield(data, space=None)
    """
    cdef HeightfieldData data

    def __cinit__(self, HeightfieldData data not None,
                placeable=True, space=None):
        cdef SpaceBase sp
        cdef dSpaceID sid

        self.data = data

        sid=NULL
        if space!=None:
            sp = space
            sid = sp.sid
        self.gid = dCreateHeightfield(sid, data.hfdid, <int>placeable)

        _geom_c2py_lut[<long>self.gid] = self

    def __init__(self, HeightfieldData data not None, space=None):
        self.space = space
        self.body = None

    def placeable(self):
        return True

    def _id(self):
        cdef long id
        id = <long>self.gid
        return id
