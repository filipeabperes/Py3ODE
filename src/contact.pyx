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


cdef class Contact:
    """This class represents a contact between two bodies in one point.

    A Contact object stores all the input parameters for a ContactJoint.
    This class wraps the ODE dContact structure which has 3 components::

     struct dContact {
       dSurfaceParameters surface;
       dContactGeom geom;
       dVector3 fdir1;
     };    

    This wrapper class provides methods to get and set the items of those
    structures.
    """
    
    cdef dContact _contact

    def __cinit__(self):
        self._contact.surface.mode = ContactBounce
        self._contact.surface.mu   = dInfinity

        self._contact.surface.bounce = 0.1

    # getMode
    def getMode(self):
        """getMode() -> flags

        Return the contact flags.
        """
        return self._contact.surface.mode

    # setMode
    def setMode(self, flags):
        """setMode(flags)

        Set the contact flags. The argument m is a combination of the
        ContactXyz flags (ContactMu2, ContactBounce, ...).
 
        @param flags: Contact flags
        @type flags: int
        """
        self._contact.surface.mode = flags

    # getMu
    def getMu(self):
        """getMu() -> float

        Return the Coulomb friction coefficient. 
        """
        return self._contact.surface.mu

    # setMu
    def setMu(self, mu):
        """setMu(mu)

        Set the Coulomb friction coefficient.

        @param mu: Coulomb friction coefficient (0..Infinity)
        @type mu: float
        """
        self._contact.surface.mu = mu

    # getMu2
    def getMu2(self):
        """getMu2() -> float

        Return the optional Coulomb friction coefficient for direction 2.
        """
        return self._contact.surface.mu2

    # setMu2
    def setMu2(self, mu):
        """setMu2(mu)

        Set the optional Coulomb friction coefficient for direction 2.

        @param mu: Coulomb friction coefficient (0..Infinity)
        @type mu: float
        """
        self._contact.surface.mu2 = mu

    # getBounce
    def getBounce(self):
        """getBounce() -> float

        Return the restitution parameter.
        """
        return self._contact.surface.bounce

    # setBounce
    def setBounce(self, b):
        """setBounce(b)

        @param b: Restitution parameter (0..1)
        @type b: float
        """
        self._contact.surface.bounce = b

    # getBounceVel
    def getBounceVel(self):
        """getBounceVel() -> float

        Return the minimum incoming velocity necessary for bounce.
        """
        return self._contact.surface.bounce_vel

    # setBounceVel
    def setBounceVel(self, bv):
        """setBounceVel(bv)

        Set the minimum incoming velocity necessary for bounce. Incoming
        velocities below this will effectively have a bounce parameter of 0.

        @param bv: Velocity
        @type bv: float
        """
        self._contact.surface.bounce_vel = bv

    # getSoftERP
    def getSoftERP(self):
        """getSoftERP() -> float

        Return the contact normal "softness" parameter.
        """
        return self._contact.surface.soft_erp

    # setSoftERP
    def setSoftERP(self, erp):
        """setSoftERP(erp)

        Set the contact normal "softness" parameter.

        @param erp: Softness parameter
        @type erp: float
        """
        self._contact.surface.soft_erp = erp

    # getSoftCFM
    def getSoftCFM(self):
        """getSoftCFM() -> float

        Return the contact normal "softness" parameter.
        """
        return self._contact.surface.soft_cfm

    # setSoftCFM
    def setSoftCFM(self, cfm):
        """setSoftCFM(cfm)

        Set the contact normal "softness" parameter.

        @param cfm: Softness parameter
        @type cfm: float
        """
        self._contact.surface.soft_cfm = cfm

    # getMotion1
    def getMotion1(self):
        """getMotion1() -> float

        Get the surface velocity in friction direction 1.
        """
        return self._contact.surface.motion1

    # setMotion1
    def setMotion1(self, m):
        """setMotion1(m)

        Set the surface velocity in friction direction 1.

        @param m: Surface velocity
        @type m: float
        """
        self._contact.surface.motion1 = m

    # getMotion2
    def getMotion2(self):
        """getMotion2() -> float

        Get the surface velocity in friction direction 2.
        """
        return self._contact.surface.motion2

    # setMotion2
    def setMotion2(self, m):
        """setMotion2(m)

        Set the surface velocity in friction direction 2.

        @param m: Surface velocity
        @type m: float
        """
        self._contact.surface.motion2 = m

    # getSlip1
    def getSlip1(self):
        """getSlip1() -> float

        Get the coefficient of force-dependent-slip (FDS) for friction
        direction 1.
        """
        return self._contact.surface.slip1

    # setSlip1
    def setSlip1(self, s):
        """setSlip1(s)

        Set the coefficient of force-dependent-slip (FDS) for friction
        direction 1.

        @param s: FDS coefficient
        @type s: float
        """
        self._contact.surface.slip1 = s

    # getSlip2
    def getSlip2(self):
        """getSlip2() -> float

        Get the coefficient of force-dependent-slip (FDS) for friction
        direction 2.
        """
        return self._contact.surface.slip2

    # setSlip2
    def setSlip2(self, s):
        """setSlip2(s)

        Set the coefficient of force-dependent-slip (FDS) for friction
        direction 1.

        @param s: FDS coefficient
        @type s: float
        """
        self._contact.surface.slip2 = s

    # getFDir1
    def getFDir1(self):
        """getFDir1() -> (x, y, z)

        Get the "first friction direction" vector that defines a direction
        along which frictional force is applied.
        """
        return (self._contact.fdir1[0], self._contact.fdir1[1], self._contact.fdir1[2])

    # setFDir1
    def setFDir1(self, fdir):
        """setFDir1(fdir)

        Set the "first friction direction" vector that defines a direction
        along which frictional force is applied. It must be of unit length
        and perpendicular to the contact normal (so it is typically
        tangential to the contact surface).

        @param fdir: Friction direction
        @type fdir: 3-sequence of floats
        """
        self._contact.fdir1[0] = fdir[0]
        self._contact.fdir1[1] = fdir[1]
        self._contact.fdir1[2] = fdir[2]
        

    # getContactGeomParams
    def getContactGeomParams(self):
        """getContactGeomParams() -> (pos, normal, depth, geom1, geom2)

        Get the ContactGeom structure of the contact.

        The return value is a tuple (pos, normal, depth, geom1, geom2)
        where pos and normal are 3-tuples of floats and depth is a single
        float. geom1 and geom2 are the Geom objects of the geoms in contact.
        """
        cdef long id1, id2

        pos = (self._contact.geom.pos[0], self._contact.geom.pos[1], self._contact.geom.pos[2])
        normal = (self._contact.geom.normal[0], self._contact.geom.normal[1], self._contact.geom.normal[2])
        depth = self._contact.geom.depth

        id1 = <long>self._contact.geom.g1
        id2 = <long>self._contact.geom.g2
        g1 = _geom_c2py_lut[id1]
        g2 = _geom_c2py_lut[id2]
        return (pos,normal,depth,g1,g2)

    # setContactGeomParams
    def setContactGeomParams(self, pos, normal, depth, g1=None, g2=None):
        """setContactGeomParams(pos, normal, depth, geom1=None, geom2=None)
        
        Set the ContactGeom structure of the contact.

        @param pos:  Contact position, in global coordinates
        @type pos: 3-sequence of floats
        @param normal: Unit length normal vector
        @type normal: 3-sequence of floats
        @param depth: Depth to which the two bodies inter-penetrate
        @type depth: float
        @param geom1: Geometry object 1 that collided
        @type geom1: Geom
        @param geom2: Geometry object 2 that collided
        @type geom2: Geom
        """

        cdef long id

        self._contact.geom.pos[0] = pos[0]
        self._contact.geom.pos[1] = pos[1]
        self._contact.geom.pos[2] = pos[2]
        self._contact.geom.normal[0] = normal[0]
        self._contact.geom.normal[1] = normal[1]
        self._contact.geom.normal[2] = normal[2]
        self._contact.geom.depth = depth
        if g1!=None:
            id = g1._id()
            self._contact.geom.g1 = <dGeomID>id
        else:
            self._contact.geom.g1 = <dGeomID>0
            
        if g2!=None:
            id = g2._id()
            self._contact.geom.g2 = <dGeomID>id
        else:
            self._contact.geom.g2 = <dGeomID>0
