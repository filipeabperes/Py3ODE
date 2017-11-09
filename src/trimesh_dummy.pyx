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

# These classes are included by the file _trimesh_switch.pyx if the
# variable TRIMESH_SUPPORT was set to False in the setup script.


cdef class TriMeshData:
    """This class stores the mesh data.

    This is only a dummy class that's used when trimesh support was disabled.
    """

    def __init__(self):
        raise NotImplementedError, "Trimesh support is disabled"


cdef class GeomTriMesh(GeomObject):
    """Trimesh object.
    
    This is only a dummy class that's used when trimesh support was disabled.
    """

    def __init__(self, TriMeshData data not None, space=None):
        raise NotImplementedError, "Trimesh support is disabled"
