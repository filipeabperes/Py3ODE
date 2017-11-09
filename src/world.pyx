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

# World
cdef class World:
    """Dynamics world.
    
    The world object is a container for rigid bodies and joints.
    
    
    Constructor::
    
      World()
    """

    cdef dWorldID wid

    def __cinit__(self):
        self.wid = dWorldCreate()

    def __dealloc__(self):
        if self.wid!=NULL:
            dWorldDestroy(self.wid)

    # setGravity
    def setGravity(self, gravity):
        """setGravity(gravity)

        Set the world's global gravity vector.

        @param gravity: Gravity vector
        @type gravity: 3-sequence of floats
        """
        dWorldSetGravity(self.wid, gravity[0], gravity[1], gravity[2])

    # getGravity
    def getGravity(self):
        """getGravity() -> 3-tuple

        Return the world's global gravity vector as a 3-tuple of floats.
        """
        cdef dVector3 g
        dWorldGetGravity(self.wid, g)
        return (g[0],g[1],g[2])

    # setERP
    def setERP(self, erp):
        """setERP(erp)

        Set the global ERP value, that controls how much error
        correction is performed in each time step. Typical values are
        in the range 0.1-0.8. The default is 0.2.

        @param erp: Global ERP value
        @type erp: float
        """
        dWorldSetERP(self.wid, erp)

    # getERP
    def getERP(self):
        """getERP() -> float

        Get the global ERP value, that controls how much error
        correction is performed in each time step. Typical values are
        in the range 0.1-0.8. The default is 0.2.
        """
        return dWorldGetERP(self.wid)

    # setCFM
    def setCFM(self, cfm):
        """setCFM(cfm)

        Set the global CFM (constraint force mixing) value. Typical
        values are in the range 10E-9 - 1. The default is 10E-5 if
        single precision is being used, or 10E-10 if double precision
        is being used.

        @param cfm: Constraint force mixing value
        @type cfm: float
        """
        dWorldSetCFM(self.wid, cfm)

    # getCFM
    def getCFM(self):
        """getCFM() -> float

        Get the global CFM (constraint force mixing) value. Typical
        values are in the range 10E-9 - 1. The default is 10E-5 if
        single precision is being used, or 10E-10 if double precision
        is being used.
        """
        return dWorldGetCFM(self.wid)

    # step
    def step(self, stepsize):
        """step(stepsize)

        Step the world. This uses a "big matrix" method that takes
        time on the order of O(m3) and memory on the order of O(m2), where m
        is the total number of constraint rows.

        For large systems this will use a lot of memory and can be
        very slow, but this is currently the most accurate method.

        @param stepsize: Time step
        @type stepsize: float
        """

        dWorldStep(self.wid, stepsize)

    # quickStep
    def quickStep(self, stepsize):
        """quickStep(stepsize)
        
        Step the world. This uses an iterative method that takes time
        on the order of O(m*N) and memory on the order of O(m), where m is
        the total number of constraint rows and N is the number of
        iterations.

        For large systems this is a lot faster than dWorldStep, but it
        is less accurate.

        @param stepsize: Time step
        @type stepsize: float        
        """
        dWorldQuickStep(self.wid, stepsize)

    # setQuickStepNumIterations
    def setQuickStepNumIterations(self, num):
        """setQuickStepNumIterations(num)
        
        Set the number of iterations that the QuickStep method
        performs per step. More iterations will give a more accurate
        solution, but will take longer to compute. The default is 20
        iterations.

        @param num: Number of iterations
        @type num: int
        """
        
        dWorldSetQuickStepNumIterations(self.wid, num)

    # getQuickStepNumIterations
    def getQuickStepNumIterations(self):
        """getQuickStepNumIterations() -> int
        
        Get the number of iterations that the QuickStep method
        performs per step. More iterations will give a more accurate
        solution, but will take longer to compute. The default is 20
        iterations.
        """
        return dWorldGetQuickStepNumIterations(self.wid)

    # setQuickStepNumIterations
    def setContactMaxCorrectingVel(self, vel):
        """setContactMaxCorrectingVel(vel)

        Set the maximum correcting velocity that contacts are allowed
        to generate. The default value is infinity (i.e. no
        limit). Reducing this value can help prevent "popping" of
        deeply embedded objects.

        @param vel: Maximum correcting velocity
        @type vel: float
        """
        dWorldSetContactMaxCorrectingVel(self.wid, vel)

    # getQuickStepNumIterations
    def getContactMaxCorrectingVel(self):
        """getContactMaxCorrectingVel() -> float

        Get the maximum correcting velocity that contacts are allowed
        to generate. The default value is infinity (i.e. no
        limit). Reducing this value can help prevent "popping" of
        deeply embedded objects.        
        """
        return dWorldGetContactMaxCorrectingVel(self.wid)

    # setContactSurfaceLayer
    def setContactSurfaceLayer(self, depth):
        """setContactSurfaceLayer(depth)

        Set the depth of the surface layer around all geometry
        objects. Contacts are allowed to sink into the surface layer
        up to the given depth before coming to rest. The default value
        is zero. Increasing this to some small value (e.g. 0.001) can
        help prevent jittering problems due to contacts being
        repeatedly made and broken.

        @param depth: Surface layer depth
        @type depth: float
        """
        dWorldSetContactSurfaceLayer(self.wid, depth)

    # getContactSurfaceLayer
    def getContactSurfaceLayer(self):
        """getContactSurfaceLayer()

        Get the depth of the surface layer around all geometry
        objects. Contacts are allowed to sink into the surface layer
        up to the given depth before coming to rest. The default value
        is zero. Increasing this to some small value (e.g. 0.001) can
        help prevent jittering problems due to contacts being
        repeatedly made and broken.
        """
        return dWorldGetContactSurfaceLayer(self.wid)

    # setAutoDisableFlag
    def setAutoDisableFlag(self, flag):
        """setAutoDisableFlag(flag)
        
        Set the default auto-disable flag for newly created bodies.

        @param flag: True = Do auto disable
        @type flag: bool
        """
        dWorldSetAutoDisableFlag(self.wid, flag)
        
    # getAutoDisableFlag
    def getAutoDisableFlag(self):
        """getAutoDisableFlag() -> bool
        
        Get the default auto-disable flag for newly created bodies.
        """
        return dWorldGetAutoDisableFlag(self.wid)
        

    # setAutoDisableLinearThreshold
    def setAutoDisableLinearThreshold(self, threshold):
        """setAutoDisableLinearThreshold(threshold)
        
        Set the default auto-disable linear threshold for newly created
        bodies.

        @param threshold: Linear threshold
        @type threshold: float
        """
        dWorldSetAutoDisableLinearThreshold(self.wid, threshold)

    # getAutoDisableLinearThreshold
    def getAutoDisableLinearThreshold(self):
        """getAutoDisableLinearThreshold() -> float
        
        Get the default auto-disable linear threshold for newly created
        bodies.
        """
        return dWorldGetAutoDisableLinearThreshold(self.wid)

    # setAutoDisableAngularThreshold
    def setAutoDisableAngularThreshold(self, threshold):
        """setAutoDisableAngularThreshold(threshold)
        
        Set the default auto-disable angular threshold for newly created
        bodies.

        @param threshold: Angular threshold
        @type threshold: float
        """
        dWorldSetAutoDisableAngularThreshold(self.wid, threshold)

    # getAutoDisableAngularThreshold
    def getAutoDisableAngularThreshold(self):
        """getAutoDisableAngularThreshold() -> float
        
        Get the default auto-disable angular threshold for newly created
        bodies.
        """
        return dWorldGetAutoDisableAngularThreshold(self.wid)
    
    # setAutoDisableSteps
    def setAutoDisableSteps(self, steps):
        """setAutoDisableSteps(steps)
        
        Set the default auto-disable steps for newly created bodies.

        @param steps: Auto disable steps
        @type steps: int
        """
        dWorldSetAutoDisableSteps(self.wid, steps)

    # getAutoDisableSteps
    def getAutoDisableSteps(self):
        """getAutoDisableSteps() -> int
        
        Get the default auto-disable steps for newly created bodies.
        """
        return dWorldGetAutoDisableSteps(self.wid)

    # setAutoDisableTime
    def setAutoDisableTime(self, time):
        """setAutoDisableTime(time)
        
        Set the default auto-disable time for newly created bodies.

        @param time: Auto disable time
        @type time: float
        """
        dWorldSetAutoDisableTime(self.wid, time)

    # getAutoDisableTime
    def getAutoDisableTime(self):
        """getAutoDisableTime() -> float
        
        Get the default auto-disable time for newly created bodies.
        """
        return dWorldGetAutoDisableTime(self.wid)

    # setLinearDamping
    def setLinearDamping(self, scale):
        """setLinearDamping(scale)

        Set the world's linear damping scale.
				@param scale The linear damping scale that is to be applied to bodies.
				Default is 0 (no damping). Should be in the interval [0, 1].
        @type scale: float
        """
        dWorldSetLinearDamping(self.wid, scale)

    # getLinearDamping
    def getLinearDamping(self):
        """getLinearDamping() -> float

        Get the world's linear damping scale.
        """
        return dWorldGetLinearDamping(self.wid)

    # setAngularDamping
    def setAngularDamping(self, scale):
        """setAngularDamping(scale)

        Set the world's angular damping scale.
				@param scale The angular damping scale that is to be applied to bodies.
				Default is 0 (no damping). Should be in the interval [0, 1].
        @type scale: float
        """
        dWorldSetAngularDamping(self.wid, scale)

    # getAngularDamping
    def getAngularDamping(self):
        """getAngularDamping() -> float

        Get the world's angular damping scale.
        """
        return dWorldGetAngularDamping(self.wid)

    # impulseToForce
    def impulseToForce(self, stepsize, impulse):
        """impulseToForce(stepsize, impulse) -> 3-tuple

        If you want to apply a linear or angular impulse to a rigid
        body, instead of a force or a torque, then you can use this
        function to convert the desired impulse into a force/torque
        vector before calling the dBodyAdd... function.

        @param stepsize: Time step
        @param impulse: Impulse vector
        @type stepsize: float
        @type impulse: 3-tuple of floats
        """
        cdef dVector3 force
        dWorldImpulseToForce(self.wid, stepsize, impulse[0], impulse[1], impulse[2], force)
        return (force[0], force[1], force[2])

    # createBody
#    def createBody(self):
#        return Body(self)

    # createBallJoint
#    def createBallJoint(self, jointgroup=None):
#        return BallJoint(self, jointgroup)

    # createHingeJoint
#    def createHingeJoint(self, jointgroup=None):
#        return HingeJoint(self, jointgroup)

    # createHinge2Joint
#    def createHinge2Joint(self, jointgroup=None):
#        return Hinge2Joint(self, jointgroup)

    # createSliderJoint
#    def createSliderJoint(self, jointgroup=None):
#        return SliderJoint(self, jointgroup)

    # createFixedJoint
#    def createFixedJoint(self, jointgroup=None):
#        return FixedJoint(self, jointgroup)

    # createContactJoint
#    def createContactJoint(self, jointgroup, contact):
#        return ContactJoint(self, jointgroup, contact)
