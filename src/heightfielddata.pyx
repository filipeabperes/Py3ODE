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

cdef class HeightfieldData:
    """This class is used to store heightfield data.
    """
    cdef dHeightfieldDataID hfdid
    # This attribute stores the tuple which gets passed to the height
    # callback. If we don't keep a reference, the tuple gets garbage
    # collected.
    cdef object calltup

    def __cinit__(self):
        self.hfdid = dGeomHeightfieldDataCreate()

    def __dealloc__(self):
        if self.hfdid!=NULL:
            dGeomHeightfieldDataDestroy(self.hfdid)

    def build_callback(self, userdata, callback, width, depth, wsamp, dsamp,
                       scale, offset, thickness, bwrap):
        cdef object tup
        cdef void* data
        tup = (callback, userdata)
        self.calltup = tup
        data = <void*>tup
        dGeomHeightfieldDataBuildCallback(self.hfdid,
                                          data, get_height, width, depth,
                                          wsamp, dsamp, scale, offset,
                                          thickness, bwrap)

cdef dReal get_height(void *data, int x, int z):
    cdef object tup
    tup = <object>data
    callback, arg = tup
    return callback(arg, x, z)
