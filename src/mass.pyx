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

cdef class Mass:
    """Mass parameters of a rigid body.

    This class stores mass parameters of a rigid body which can be
    accessed through the following attributes:

     - mass: The total mass of the body (float)
     - c:    The center of gravity position in body frame (3-tuple of floats)
     - I:    The 3x3 inertia tensor in body frame (3-tuple of 3-tuples)

    This class wraps the dMass structure from the C API.

    @ivar mass: The total mass of the body
    @ivar c: The center of gravity position in body frame (cx, cy, cz)
    @ivar I: The 3x3 inertia tensor in body frame ((I11, I12, I13), (I12, I22, I23), (I13, I23, I33))
    @type mass: float
    @type c: 3-tuple of floats
    @type I: 3-tuple of 3-tuples of floats 
    """
    cdef dMass _mass

    def __cinit__(self):
        dMassSetZero(&self._mass)

    def setZero(self):
        """setZero()

        Set all the mass parameters to zero."""
        dMassSetZero(&self._mass)

    def setParameters(self, mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23):
        """setParameters(mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23)

        Set the mass parameters to the given values.

        @param mass: Total mass of the body.
        @param cgx: Center of gravity position in the body frame (x component).
        @param cgy: Center of gravity position in the body frame (y component).
        @param cgz: Center of gravity position in the body frame (z component).
        @param I11: Inertia tensor
        @param I22: Inertia tensor
        @param I33: Inertia tensor
        @param I12: Inertia tensor
        @param I13: Inertia tensor
        @param I23: Inertia tensor
        @type mass: float
        @type cgx: float
        @type cgy: float
        @type cgz: float
        @type I11: float
        @type I22: float
        @type I33: float
        @type I12: float
        @type I13: float
        @type I23: float
        """
        dMassSetParameters(&self._mass, mass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23)

    def setSphere(self, density, radius):
        """setSphere(density, radius)
        
        Set the mass parameters to represent a sphere of the given radius
        and density, with the center of mass at (0,0,0) relative to the body.

        @param density: The density of the sphere
        @param radius: The radius of the sphere
        @type density: float
        @type radius: float
        """
        dMassSetSphere(&self._mass, density, radius)

    def setSphereTotal(self, total_mass, radius):
        """setSphereTotal(total_mass, radius)
        
        Set the mass parameters to represent a sphere of the given radius
        and mass, with the center of mass at (0,0,0) relative to the body.

        @param total_mass: The total mass of the sphere
        @param radius: The radius of the sphere
        @type total_mass: float
        @type radius: float
        """
        dMassSetSphere(&self._mass, total_mass, radius)

    def setCappedCylinder(self, density, direction, r, h):
        """setCappedCylinder(density, direction, r, h)
        
        Set the mass parameters to represent a capped cylinder of the
        given parameters and density, with the center of mass at
        (0,0,0) relative to the body. The radius of the cylinder (and
        the spherical cap) is r. The length of the cylinder (not
        counting the spherical cap) is h. The cylinder's long axis is
        oriented along the body's x, y or z axis according to the
        value of direction (1=x, 2=y, 3=z).

        @param density: The density of the cylinder
        @param direction: The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)
        @param r: The radius of the cylinder
        @param h: The length of the cylinder (without the caps)
        @type density: float
        @type direction: int
        @type r: float
        @type h: float
        """
        dMassSetCappedCylinder(&self._mass, density, direction, r, h)

    def setCappedCylinderTotal(self, total_mass, direction, r, h):
        """setCappedCylinderTotal(total_mass, direction, r, h)
        
        Set the mass parameters to represent a capped cylinder of the
        given parameters and mass, with the center of mass at
        (0,0,0) relative to the body. The radius of the cylinder (and
        the spherical cap) is r. The length of the cylinder (not
        counting the spherical cap) is h. The cylinder's long axis is
        oriented along the body's x, y or z axis according to the
        value of direction (1=x, 2=y, 3=z).

        @param total_mass: The total mass of the cylinder
        @param direction: The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)
        @param r: The radius of the cylinder
        @param h: The length of the cylinder (without the caps)
        @type total_mass: float
        @type direction: int
        @type r: float
        @type h: float
        """
        dMassSetCappedCylinderTotal(&self._mass, total_mass, direction, r, h)

    def setCylinder(self, density, direction, r, h):
        """setCylinder(density, direction, r, h)
        
        Set the mass parameters to represent a flat-ended cylinder of
        the given parameters and density, with the center of mass at
        (0,0,0) relative to the body. The radius of the cylinder is r.
        The length of the cylinder is h. The cylinder's long axis is
        oriented along the body's x, y or z axis according to the value
        of direction (1=x, 2=y, 3=z).

        @param density: The density of the cylinder
        @param direction: The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)
        @param r: The radius of the cylinder
        @param h: The length of the cylinder
        @type density: float
        @type direction: int
        @type r: float
        @type h: float
        """
        dMassSetCylinder(&self._mass, density, direction, r, h)

    def setCylinderTotal(self, total_mass, direction, r, h):
        """setCylinderTotal(total_mass, direction, r, h)
        
        Set the mass parameters to represent a flat-ended cylinder of
        the given parameters and mass, with the center of mass at
        (0,0,0) relative to the body. The radius of the cylinder is r.
        The length of the cylinder is h. The cylinder's long axis is
        oriented along the body's x, y or z axis according to the value
        of direction (1=x, 2=y, 3=z).

        @param total_mass: The total mass of the cylinder
        @param direction: The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)
        @param r: The radius of the cylinder
        @param h: The length of the cylinder
        @type total_mass: float
        @type direction: int
        @type r: float
        @type h: float
        """
        dMassSetCylinderTotal(&self._mass, total_mass, direction, r, h)

    def setBox(self, density, lx, ly, lz):
        """setBox(density, lx, ly, lz)

        Set the mass parameters to represent a box of the given
        dimensions and density, with the center of mass at (0,0,0)
        relative to the body. The side lengths of the box along the x,
        y and z axes are lx, ly and lz.

        @param density: The density of the box
        @param lx: The length along the x axis
        @param ly: The length along the y axis
        @param lz: The length along the z axis
        @type density: float
        @type lx: float
        @type ly: float
        @type lz: float
        """
        dMassSetBox(&self._mass, density, lx, ly, lz)

    def setBoxTotal(self, total_mass, lx, ly, lz):
        """setBoxTotal(total_mass, lx, ly, lz)

        Set the mass parameters to represent a box of the given
        dimensions and mass, with the center of mass at (0,0,0)
        relative to the body. The side lengths of the box along the x,
        y and z axes are lx, ly and lz.

        @param total_mass: The total mass of the box
        @param lx: The length along the x axis
        @param ly: The length along the y axis
        @param lz: The length along the z axis
        @type total_mass: float
        @type lx: float
        @type ly: float
        @type lz: float
        """
        dMassSetBoxTotal(&self._mass, total_mass, lx, ly, lz)

    def adjust(self, newmass):
        """adjust(newmass)

        Adjust the total mass. Given mass parameters for some object,
        adjust them so the total mass is now newmass. This is useful
        when using the setXyz() methods to set the mass parameters for
        certain objects - they take the object density, not the total
        mass.

        @param newmass: The new total mass
        @type newmass: float
        """
        dMassAdjust(&self._mass, newmass)

    def translate(self, t):
        """translate(t)

        Adjust mass parameters. Given mass parameters for some object,
        adjust them to represent the object displaced by (x,y,z)
        relative to the body frame.

        @param t: Translation vector (x, y, z)
        @type t: 3-tuple of floats
        """
        dMassTranslate(&self._mass, t[0], t[1], t[2])

#    def rotate(self, R):
#        """
#        Given mass parameters for some object, adjust them to
#        represent the object rotated by R relative to the body frame.
#        """
#        pass

    def add(self, Mass b):
        """add(b)

        Add the mass b to the mass object. Masses can also be added using
        the + operator.

        @param b: The mass to add to this mass
        @type b: Mass
        """
        dMassAdd(&self._mass, &b._mass)

    def __getattr__(self, name):
        if name=="mass":
            return self._mass.mass
        elif name=="c":
            return (self._mass.c[0], self._mass.c[1], self._mass.c[2])
        elif name=="I":
            return ((self._mass.I[0],self._mass.I[1],self._mass.I[2]),
                    (self._mass.I[4],self._mass.I[5],self._mass.I[6]),
                    (self._mass.I[8],self._mass.I[9],self._mass.I[10]))
        else:
            raise AttributeError,"Mass object has no attribute '"+name+"'"

    def __setattr__(self, name, value):
        if name=="mass":
            self.adjust(value)
        elif name=="c":
            raise AttributeError,"Use the setParameter() method to change c"
        elif name=="I":
            raise AttributeError,"Use the setParameter() method to change I"
        else:
            raise AttributeError,"Mass object has no attribute '"+name+"'"

    def __add__(self, Mass b):
        self.add(b)
        return self

    def __str__(self):
        m   = str(self._mass.mass)
        sc0 = str(self._mass.c[0])
        sc1 = str(self._mass.c[1])
        sc2 = str(self._mass.c[2])
        I11 = str(self._mass.I[0])
        I22 = str(self._mass.I[5])
        I33 = str(self._mass.I[10])
        I12 = str(self._mass.I[1])
        I13 = str(self._mass.I[2])
        I23 = str(self._mass.I[6])
        return "Mass=%s\nCg=(%s, %s, %s)\nI11=%s I22=%s I33=%s\nI12=%s I13=%s I23=%s"%(m,sc0,sc1,sc2,I11,I22,I33,I12,I13,I23)
#        return "Mass=%s / Cg=(%s, %s, %s) / I11=%s I22=%s I33=%s I12=%s I13=%s I23=%s"%(m,sc0,sc1,sc2,I11,I22,I33,I12,I13,I23)
