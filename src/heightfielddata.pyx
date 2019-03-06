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

    The dHeightfieldData is a storage class, similar to the dTrimeshData class,
    that holds all geom properties and optionally height sample data.

    NOTE:
    Heightfields treat Y as the "UP" axis.
    """
    cdef dHeightfieldDataID hfdid
    # This attribute stores the tuple which gets passed to the height
    # callback. If we don't keep a reference, the tuple gets garbage
    # collected.
    cdef object calltup

    def __cinit__(self):
        self.hfdid = dGeomHeightfieldDataCreate()

    def __dealloc__(self):
        if self.hfdid != NULL:
            dGeomHeightfieldDataDestroy(self.hfdid)

    def buildByte(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap):
        """buildByte(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap)

        Configures a dHeightfieldDataID to use height data in byte format.

        Before a dHeightfieldDataID can be used by a geom it must be configured
        to specify the format of the height data.

        This call specifies that the heightfield data is stored as a rectangular
        array of bytes (8 bit unsigned) representing the height at each sample point.

        @param heightData: A list of len( widthSamples * depthSamples ) containing the height values.
            NOTE: This NEEDS to be a contigious array (easy to create one with
            numpy, see tutorial_heightmap.py) otherwise ODE won't be able to
            access it correctly, and it will likely lead to memory corruption.
            There isn't an automatic conversion built in here as that would bring
            another - often unnecessary - dependency into Py3ODE.
        @param copy: True / False, should ODE copy the heightData into it's
            internal memory or will it remain accessible (not garbage collected)
            for the lifetime of the heightfield?
        @param width: Specifies the total 'width' of the heightfield along the geom's local x axis.
        @param depth: Specifies the total 'depth' of the heightfield along the geom's local z axis.
        @param widthSamples: Specifies the number of vertices to sample along the
            width of the heightfield. Each vertex has a corresponding height
            value which forms the overall shape.
            Naturally this value must be at least two or more.
        @param depthSamples: Specifies the number of vertices to sample
            along the depth of the heightfield.
        @param scale: A uniform scale applied to all raw height data.
        @param offset: An offset applied to the scaled height data.
        @param thickness: A value subtracted from the lowest height
            value which in effect adds an additional cuboid to the base of the
            heightfield. This is used to prevent geoms from looping under the
            desired terrain and not registering as a collision. Note that the
            thickness is not affected by the scale or offset parameters.
        @param bWrap: If non-zero the heightfield will infinitely tile in both
            directions along the local x and z axes. If zero the heightfield is
            bounded from zero to width in the local x axis, and zero to depth in
            the local z axis.
        """
        cdef unsigned char[::1] memview = heightData
        dGeomHeightfieldDataBuildByte(self.hfdid, &memview[0], <int> copy,
            width, depth, widthSamples, depthSamples, scale, offset, thickness, <int> bWrap)

    def buildShort(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap):
        """buildShort(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap)

        Configures a dHeightfieldDataID to use height data in byte format.

        Before a dHeightfieldDataID can be used by a geom it must be configured
        to specify the format of the height data.

        This call specifies that the heightfield data is stored as a rectangular
        array of shorts (16 bit signed) representing the height at each sample point.

        @param heightData: A list of len( widthSamples * depthSamples ) containing the height values.
            NOTE: This NEEDS to be a contigious array (easy to create one with
            numpy, see tutorial_heightmap.py) otherwise ODE won't be able to
            access it correctly, and it will likely lead to memory corruption.
            There isn't an automatic conversion built in here as that would bring
            another - often unnecessary - dependency into Py3ODE.
        @param copy: True / False, should ODE copy the heightData into it's
            internal memory or will it remain accessible (not garbage collected)
            for the lifetime of the heightfield?
        @param width: Specifies the total 'width' of the heightfield along the geom's local x axis.
        @param depth: Specifies the total 'depth' of the heightfield along the geom's local z axis.
        @param widthSamples: Specifies the number of vertices to sample along the
            width of the heightfield. Each vertex has a corresponding height
            value which forms the overall shape.
            Naturally this value must be at least two or more.
        @param depthSamples: Specifies the number of vertices to sample
            along the depth of the heightfield.
        @param scale: A uniform scale applied to all raw height data.
        @param offset: An offset applied to the scaled height data.
        @param thickness: A value subtracted from the lowest height
            value which in effect adds an additional cuboid to the base of the
            heightfield. This is used to prevent geoms from looping under the
            desired terrain and not registering as a collision. Note that the
            thickness is not affected by the scale or offset parameters.
        @param bWrap: If non-zero the heightfield will infinitely tile in both
            directions along the local x and z axes. If zero the heightfield is
            bounded from zero to width in the local x axis, and zero to depth in
            the local z axis.
        """
        cdef short[::1] memview = heightData # yes this isn't unsigned - no IDK why
        dGeomHeightfieldDataBuildShort(self.hfdid, &memview[0], <int> copy,
            width, depth, widthSamples, depthSamples, scale, offset, thickness, <int> bWrap)

    def buildSingle(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap):
        """buildSingle(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap)

        Configures a dHeightfieldDataID to use height data in byte format.

        Before a dHeightfieldDataID can be used by a geom it must be configured
        to specify the format of the height data.

        This call specifies that the heightfield data is stored as a rectangular
        array of single precision floats representing the height at each sample point.

        @param heightData: A list of len( widthSamples * depthSamples ) containing the height values.
            NOTE: This NEEDS to be a contigious array (easy to create one with
            numpy, see tutorial_heightmap.py) otherwise ODE won't be able to
            access it correctly, and it will likely lead to memory corruption.
            There isn't an automatic conversion built in here as that would bring
            another - often unnecessary - dependency into Py3ODE.
        @param copy: True / False, should ODE copy the heightData into it's
            internal memory or will it remain accessible (not garbage collected)
            for the lifetime of the heightfield?
        @param width: Specifies the total 'width' of the heightfield along the geom's local x axis.
        @param depth: Specifies the total 'depth' of the heightfield along the geom's local z axis.
        @param widthSamples: Specifies the number of vertices to sample along the
            width of the heightfield. Each vertex has a corresponding height
            value which forms the overall shape.
            Naturally this value must be at least two or more.
        @param depthSamples: Specifies the number of vertices to sample
            along the depth of the heightfield.
        @param scale: A uniform scale applied to all raw height data.
        @param offset: An offset applied to the scaled height data.
        @param thickness: A value subtracted from the lowest height
            value which in effect adds an additional cuboid to the base of the
            heightfield. This is used to prevent geoms from looping under the
            desired terrain and not registering as a collision. Note that the
            thickness is not affected by the scale or offset parameters.
        @param bWrap: If non-zero the heightfield will infinitely tile in both
            directions along the local x and z axes. If zero the heightfield is
            bounded from zero to width in the local x axis, and zero to depth in
            the local z axis.
        """
        cdef float[::1] memview = heightData
        dGeomHeightfieldDataBuildSingle(self.hfdid, &memview[0], <int> copy,
            width, depth, widthSamples, depthSamples, scale, offset, thickness, <int> bWrap)

    def buildDouble(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap):
        """buildDouble(self, heightData, copy, width, depth, int widthSamples, int depthSamples, scale, offset, thickness, bWrap)

        Configures a dHeightfieldDataID to use height data in byte format.

        Before a dHeightfieldDataID can be used by a geom it must be configured
        to specify the format of the height data.

        This call specifies that the heightfield data is stored as a rectangular
        array of double precision floats representing the height at each sample point.

        @param heightData: A list of len( widthSamples * depthSamples ) containing the height values.
            NOTE: This NEEDS to be a contigious array (easy to create one with
            numpy, see tutorial_heightmap.py) otherwise ODE won't be able to
            access it correctly, and it will likely lead to memory corruption.
            There isn't an automatic conversion built in here as that would bring
            another - often unnecessary - dependency into Py3ODE.
        @param copy: True / False, should ODE copy the heightData into it's
            internal memory or will it remain accessible (not garbage collected)
            for the lifetime of the heightfield?
        @param width: Specifies the total 'width' of the heightfield along the geom's local x axis.
        @param depth: Specifies the total 'depth' of the heightfield along the geom's local z axis.
        @param widthSamples: Specifies the number of vertices to sample along the
            width of the heightfield. Each vertex has a corresponding height
            value which forms the overall shape.
            Naturally this value must be at least two or more.
        @param depthSamples: Specifies the number of vertices to sample
            along the depth of the heightfield.
        @param scale: A uniform scale applied to all raw height data.
        @param offset: An offset applied to the scaled height data.
        @param thickness: A value subtracted from the lowest height
            value which in effect adds an additional cuboid to the base of the
            heightfield. This is used to prevent geoms from looping under the
            desired terrain and not registering as a collision. Note that the
            thickness is not affected by the scale or offset parameters.
        @param bWrap: If non-zero the heightfield will infinitely tile in both
            directions along the local x and z axes. If zero the heightfield is
            bounded from zero to width in the local x axis, and zero to depth in
            the local z axis.
        """
        cdef double[::1] memview = heightData
        dGeomHeightfieldDataBuildDouble(self.hfdid, &memview[0], <int> copy,
            width, depth, widthSamples, depthSamples, scale, offset, thickness, <int> bWrap)

    def setBounds(self, float min_height, float max_height):
        """setBounds(min_height, max_height)

        Manually set the minimum and maximum height bounds.

        This call allows you to set explicit min / max values after initial
        creation typically for callback heightfields which default to +/- infinity,
        or those whose data has changed. This must be set prior to binding with a
        geom, as the the AABB is not recomputed after it's first generation.

        NOTE:
        The minimum and maximum values are used to compute the AABB
        for the heightfield which is used for early rejection of collisions.
        A close fit will yield a more efficient collision check.

        @param min_height: The new minimum height value. Scale, offset and thickness is then applied.
        @param max_height: The new maximum height value. Scale and offset is then applied.
        """
        dGeomHeightfieldDataSetBounds(self.hfdid, min_height, max_height)

    def build_callback(self, userdata, callback, width, depth, widthSamples, depthSamples, scale, offset, thickness, bwrap):
        """build_callback(self, userdata, callback, width, depth, widthSamples, depthSamples, scale, offset, thickness, bwrap)

        Configures a dHeightfieldDataID to use a callback to retrieve height data.

        Before a dHeightfieldDataID can be used by a geom it must be configured
        to specify the format of the height data.

        This call specifies that the heightfield data is computed by
        the user and it should use the given callback when determining the height
        of a given element of it's shape.

        @param userdata: A python object that gets passed through to the callback function
        @param callback: A python function that does the actual computing it takes 3 arguments and returns a number epresenting the vertices height:
            def callback(arg, x, z):
                return 0
        @param width: Specifies the total 'width' of the heightfield along the geom's local x axis.
        @param depth: Specifies the total 'depth' of the heightfield along the geom's local z axis.
        @param widthSamples: Specifies the number of vertices to sample along
            the width of the heightfield. Each vertex has a corresponding height
            value which forms the overall shape.
            Naturally this value must be at least two or more.
        @param depthSamples: Specifies the number of vertices to sample along the depth of the heightfield.
        @param scale: A uniform scale applied to all raw height data.
        @param offset: An offset applied to the scaled height data.
        @param thickness: A value subtracted from the lowest height value which
            in effect adds an additional cuboid to the base of the heightfield.
            This is used to prevent geoms from looping under the desired terrain
            and not registering as a collision. Note that the thickness is not
            affected by the scale or offset parameters.
        @param bWrap: If non-zero the heightfield will infinitely tile in both
            directions along the local x and z axes. If zero the heightfield is
            bounded from zero to width in the local x axis, and zero to depth in
            the local z axis.
        """
        cdef object tup
        cdef void* data
        tup = (callback, userdata)
        self.calltup = tup
        data = <void*>tup
        dGeomHeightfieldDataBuildCallback(self.hfdid, data, get_height, width, depth,
                                          widthSamples, depthSamples, scale, offset,
                                          thickness, bwrap)


# Callback function for the dGeomHeightfieldDataBuildCallback() call in the HeightfieldData.build_callback() method
# The data parameter is a tuple (Python-Callback, Arguments).
# The function calls a Python callback function with 3 arguments:
# def callback(UserArg, x, z)
# x and z are the world coordinates of the heightmap's current vertex.
cdef dReal get_height(void *data, int x, int z):
    cdef object tup
    tup = <object>data
    callback, arg = tup
    return callback(arg, x, z)
