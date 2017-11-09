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

# Body
cdef class Body:
    """The rigid body class encapsulating the ODE body.

    This class represents a rigid body that has a location and orientation
    in space and that stores the mass properties of an object.

    When creating a Body object you have to pass the world it belongs to
    as argument to the constructor::

      >>> import ode
      >>> w = ode.World()
      >>> b = ode.Body(w)
    """

    cdef dBodyID bid
    # A reference to the world so that the world won't be destroyed while
    # there are still joints using it.
    cdef object world
    
    # A dictionary with user attributes
    # (set via __getattr__ and __setattr__)
    cdef object userattribs

    def __cinit__(self, World world not None):
        self.bid = dBodyCreate(world.wid)

    def __init__(self, World world not None):
        """Constructor.

        @param world: The world in which the body should be created.
        @type world: World
        """
        self.world = world
        self.userattribs = {}

    def __dealloc__(self):
        if self.bid!=NULL:
            dBodyDestroy(self.bid)

    def __getattr__(self, name):
        try:
            return self.userattribs[name]
        except:
            raise AttributeError, "Body object has no attribute '%s'"%name
            
    def __setattr__(self, name, value):
        self.userattribs[name] = value

    def __delattr__(self, name):
        try:
            del self.userattribs[name]
        except:
            raise AttributeError, "Body object has no attribute '%s'"%name

    # setPosition
    def setPosition(self, pos):
        """setPosition(pos)

        Set the position of the body.

        @param pos: The new position
        @type pos: 3-sequence of floats
        """
        dBodySetPosition(self.bid, pos[0], pos[1], pos[2])

    # getPosition
    def getPosition(self):
        """getPosition() -> 3-tuple

        Return the current position of the body.
        """
        cdef dReal* p
        # The "const" in the original return value is cast away
        p = <dReal*>dBodyGetPosition(self.bid)
        return (p[0],p[1],p[2])

    # setRotation
    def setRotation(self, R):
        """setRotation(R)

        Set the orientation of the body. The rotation matrix must be
        given as a sequence of 9 floats which are the elements of the
        matrix in row-major order.

        @param R: Rotation matrix
        @type R: 9-sequence of floats
        """
        cdef dMatrix3 m
        m[0] = R[0]
        m[1] = R[1]
        m[2] = R[2]
        m[3] = 0
        m[4] = R[3]
        m[5] = R[4]
        m[6] = R[5]
        m[7] = 0
        m[8] = R[6]
        m[9] = R[7]
        m[10] = R[8]
        m[11] = 0
        dBodySetRotation(self.bid, m)

    # getRotation
    def getRotation(self):
        """getRotation() -> 9-tuple

        Return the current rotation matrix as a tuple of 9 floats (row-major
        order).
        """
        cdef dReal* m
        # The "const" in the original return value is cast away
        m = <dReal*>dBodyGetRotation(self.bid)
        return (m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10])

    # getQuaternion
    def getQuaternion(self):
        """getQuaternion() -> 4-tuple

        Return the current rotation as a quaternion. The return value
        is a list of 4 floats.
        """
        cdef dReal* q
        q = <dReal*>dBodyGetQuaternion(self.bid)
        return (q[0], q[1], q[2], q[3])

    # setQuaternion
    def setQuaternion(self, q):
        """setQuaternion(q)

        Set the orientation of the body. The quaternion must be given as a
        sequence of 4 floats.

        @param q: Quaternion
        @type q: 4-sequence of floats
        """
        cdef dQuaternion w
        w[0] = q[0]
        w[1] = q[1]
        w[2] = q[2]
        w[3] = q[3]
        dBodySetQuaternion(self.bid, w)

    # setLinearVel
    def setLinearVel(self, vel):
        """setLinearVel(vel)

        Set the linear velocity of the body.

        @param vel: New velocity
        @type vel: 3-sequence of floats
        """
        dBodySetLinearVel(self.bid, vel[0], vel[1], vel[2])

    # getLinearVel
    def getLinearVel(self):
        """getLinearVel() -> 3-tuple

        Get the current linear velocity of the body.
        """
        cdef dReal* p
        # The "const" in the original return value is cast away
        p = <dReal*>dBodyGetLinearVel(self.bid)
        return (p[0],p[1],p[2])

    # setAngularVel
    def setAngularVel(self, vel):
        """setAngularVel(vel)

        Set the angular velocity of the body.

        @param vel: New angular velocity
        @type vel: 3-sequence of floats
        """
        dBodySetAngularVel(self.bid, vel[0], vel[1], vel[2])

    # getAngularVel
    def getAngularVel(self):
        """getAngularVel() -> 3-tuple

        Get the current angular velocity of the body.
        """
        cdef dReal* p
        # The "const" in the original return value is cast away
        p = <dReal*>dBodyGetAngularVel(self.bid)
        return (p[0],p[1],p[2])
    
    # setMass
    def setMass(self, Mass mass):
        """setMass(mass)

        Set the mass properties of the body. The argument mass must be
        an instance of a Mass object.

        @param mass: Mass properties
        @type mass: Mass
        """
        dBodySetMass(self.bid, &mass._mass)

    # getMass
    def getMass(self):
        """getMass() -> mass

        Return the mass properties as a Mass object.
        """
        cdef Mass m
        m=Mass()
        dBodyGetMass(self.bid, &m._mass)
        return m

    # addForce
    def addForce(self, f):
        """addForce(f)

        Add an external force f given in absolute coordinates. The force
        is applied at the center of mass.

        @param f: Force
        @type f: 3-sequence of floats
        """
        dBodyAddForce(self.bid, f[0], f[1], f[2])

    # addTorque
    def addTorque(self, t):
        """addTorque(t)

        Add an external torque t given in absolute coordinates.

        @param t: Torque
        @type t: 3-sequence of floats
        """
        dBodyAddTorque(self.bid, t[0], t[1], t[2])

    # addRelForce
    def addRelForce(self, f):
        """addRelForce(f)

        Add an external force f given in relative coordinates
        (relative to the body's own frame of reference). The force
        is applied at the center of mass.

        @param f: Force
        @type f: 3-sequence of floats
        """
        dBodyAddRelForce(self.bid, f[0], f[1], f[2])

    # addRelTorque
    def addRelTorque(self, t):
        """addRelTorque(t)

        Add an external torque t given in relative coordinates
        (relative to the body's own frame of reference).

        @param t: Torque
        @type t: 3-sequence of floats
        """
        dBodyAddRelTorque(self.bid, t[0], t[1], t[2])

    # addForceAtPos
    def addForceAtPos(self, f, p):
        """addForceAtPos(f, p)

        Add an external force f at position p. Both arguments must be
        given in absolute coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddForceAtPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # addForceAtRelPos
    def addForceAtRelPos(self, f, p):
        """addForceAtRelPos(f, p)

        Add an external force f at position p. f is given in absolute
        coordinates and p in absolute coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddForceAtRelPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # addRelForceAtPos
    def addRelForceAtPos(self, f, p):
        """addRelForceAtPos(f, p)

        Add an external force f at position p. f is given in relative
        coordinates and p in relative coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddRelForceAtPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # addRelForceAtRelPos
    def addRelForceAtRelPos(self, f, p):
        """addRelForceAtRelPos(f, p)

        Add an external force f at position p. Both arguments must be
        given in relative coordinates.

        @param f: Force
        @param p: Position
        @type f: 3-sequence of floats
        @type p: 3-sequence of floats
        """
        dBodyAddRelForceAtRelPos(self.bid, f[0], f[1], f[2], p[0], p[1], p[2])

    # getForce
    def getForce(self):
        """getForce() -> 3-tuple

        Return the current accumulated force.
        """
        cdef dReal* f
        # The "const" in the original return value is cast away
        f = <dReal*>dBodyGetForce(self.bid)
        return (f[0],f[1],f[2])

    # getTorque
    def getTorque(self):
        """getTorque() -> 3-tuple

        Return the current accumulated torque.
        """
        cdef dReal* f
        # The "const" in the original return value is cast away
        f = <dReal*>dBodyGetTorque(self.bid)
        return (f[0],f[1],f[2])

    # setForce
    def setForce(self, f):
        """setForce(f)

        Set the body force accumulation vector.

        @param f: Force
        @type f: 3-tuple of floats
        """
        dBodySetForce(self.bid, f[0], f[1], f[2])

    # setTorque
    def setTorque(self, t):
        """setTorque(t)

        Set the body torque accumulation vector.

        @param t: Torque
        @type t: 3-tuple of floats
        """
        dBodySetTorque(self.bid, t[0], t[1], t[2])

    # getRelPointPos
    def getRelPointPos(self, p):
        """getRelPointPos(p) -> 3-tuple

        Utility function that takes a point p on a body and returns
        that point's position in global coordinates. The point p
        must be given in body relative coordinates.

        @param p: Body point (local coordinates)
        @type p: 3-sequence of floats
        """

        cdef dVector3 res 
        dBodyGetRelPointPos(self.bid, p[0], p[1], p[2], res)
        return (res[0], res[1], res[2])

    # getRelPointVel
    def getRelPointVel(self, p):
        """getRelPointVel(p) -> 3-tuple

        Utility function that takes a point p on a body and returns
        that point's velocity in global coordinates. The point p
        must be given in body relative coordinates.

        @param p: Body point (local coordinates)
        @type p: 3-sequence of floats
        """
        cdef dVector3 res 
        dBodyGetRelPointVel(self.bid, p[0], p[1], p[2], res)
        return (res[0], res[1], res[2])

    # getPointVel
    def getPointVel(self, p):
        """getPointVel(p) -> 3-tuple

        Utility function that takes a point p on a body and returns
        that point's velocity in global coordinates. The point p
        must be given in global coordinates.

        @param p: Body point (global coordinates)
        @type p: 3-sequence of floats
        """
        cdef dVector3 res 
        dBodyGetPointVel(self.bid, p[0], p[1], p[2], res)
        return (res[0], res[1], res[2])

    # getPosRelPoint
    def getPosRelPoint(self, p):
        """getPosRelPoint(p) -> 3-tuple

        This is the inverse of getRelPointPos(). It takes a point p in
        global coordinates and returns the point's position in
        body-relative coordinates.

        @param p: Body point (global coordinates)
        @type p: 3-sequence of floats
        """
        cdef dVector3 res 
        dBodyGetPosRelPoint(self.bid, p[0], p[1], p[2], res)
        return (res[0], res[1], res[2])

    # vectorToWorld
    def vectorToWorld(self, v):
        """vectorToWorld(v) -> 3-tuple

        Given a vector v expressed in the body coordinate system, rotate
        it to the world coordinate system.

        @param v: Vector in body coordinate system
        @type v: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyVectorToWorld(self.bid, v[0], v[1], v[2], res)
        return (res[0], res[1], res[2])

    # vectorFromWorld
    def vectorFromWorld(self, v):
        """vectorFromWorld(v) -> 3-tuple

        Given a vector v expressed in the world coordinate system, rotate
        it to the body coordinate system.

        @param v: Vector in world coordinate system
        @type v: 3-sequence of floats
        """
        cdef dVector3 res
        dBodyVectorFromWorld(self.bid, v[0], v[1], v[2], res)
        return (res[0], res[1], res[2])        
        
        
    # Enable
    def enable(self):
        """enable()

        Manually enable a body.
        """
        dBodyEnable(self.bid)
        
    # Disable
    def disable(self):
        """disable()

        Manually disable a body. Note that a disabled body that is connected
        through a joint to an enabled body will be automatically re-enabled
        at the next simulation step.
        """
        dBodyDisable(self.bid)
        
    # isEnabled
    def isEnabled(self):
        """isEnabled() -> bool

        Check if a body is currently enabled.
        """
        return dBodyIsEnabled(self.bid)
        
    # setFiniteRotationMode
    def setFiniteRotationMode(self, mode):
        """setFiniteRotationMode(mode)

        This function controls the way a body's orientation is updated at
        each time step. The mode argument can be:
        
         - 0: An "infinitesimal" orientation update is used. This is
           fast to compute, but it can occasionally cause inaccuracies
           for bodies that are rotating at high speed, especially when
           those bodies are joined to other bodies. This is the default
           for every new body that is created.
        
         - 1: A "finite" orientation update is used. This is more
           costly to compute, but will be more accurate for high speed
           rotations. Note however that high speed rotations can result
           in many types of error in a simulation, and this mode will
           only fix one of those sources of error.

        @param mode: Rotation mode (0/1)
        @type mode: int
        """
        dBodySetFiniteRotationMode(self.bid, mode)
        
    # getFiniteRotationMode
    def getFiniteRotationMode(self):
        """getFiniteRotationMode() -> mode (0/1)

        Return the current finite rotation mode of a body (0 or 1).
        See setFiniteRotationMode().
        """
        return dBodyGetFiniteRotationMode(self.bid)

    # setFiniteRotationAxis
    def setFiniteRotationAxis(self, a):
        """setFiniteRotationAxis(a)

        Set the finite rotation axis of the body.  This axis only has a
        meaning when the finite rotation mode is set
        (see setFiniteRotationMode()).
        
        @param a: Axis
        @type a: 3-sequence of floats
        """
        dBodySetFiniteRotationAxis(self.bid, a[0], a[1], a[2])

    # getFiniteRotationAxis
    def getFiniteRotationAxis(self):
        """getFiniteRotationAxis() -> 3-tuple

        Return the current finite rotation axis of the body.
        """
        cdef dVector3 p
        # The "const" in the original return value is cast away
        dBodyGetFiniteRotationAxis(self.bid, p)
        return (p[0],p[1],p[2])
        
    # getNumJoints
    def getNumJoints(self):
        """getNumJoints() -> int

        Return the number of joints that are attached to this body.
        """
        return dBodyGetNumJoints(self.bid)

    # setGravityMode
    def setGravityMode(self, mode):
        """setGravityMode(mode)

        Set whether the body is influenced by the world's gravity
        or not. If mode is True it is, otherwise it isn't.
        Newly created bodies are always influenced by the world's gravity.

        @param mode: Gravity mode
        @type mode: bool
        """
        dBodySetGravityMode(self.bid, mode)
        
    # getGravityMode
    def getGravityMode(self):
        """getGravityMode() -> bool

        Return True if the body is influenced by the world's gravity.
        """
        return dBodyGetGravityMode(self.bid)


    def setDynamic(self):
        """setDynamic()

        Set a body to the (default) "dynamic" state, instead of "kinematic".
        See setKinematic() for more information.
        """
        dBodySetDynamic(self.bid)

    def setKinematic(self):
        """setKinematic()

        Set the kinematic state of the body (change it into a kinematic body)

        Kinematic bodies behave as if they had infinite mass. This means they don't react
        to any force (gravity, constraints or user-supplied); they simply follow 
        velocity to reach the next position. [from ODE wiki]
        """
        dBodySetKinematic(self.bid)

    def isKinematic(self):
        """isKinematic() -> bool

        Return True if the body is kinematic (not influenced by other forces).

        Kinematic bodies behave as if they had infinite mass. This means they don't react
        to any force (gravity, constraints or user-supplied); they simply follow
        velocity to reach the next position. [from ODE wiki]
        """
        return dBodyIsKinematic(self.bid)
