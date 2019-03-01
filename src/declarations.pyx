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

cdef extern from "stdlib.h":

    void* malloc(long)
    void free(void*)

cdef extern from "stdio.h":
    int printf(char*)

cdef extern from "ode/ode.h":

    ctypedef double dReal

    # Dummy structs
    cdef struct dxWorld:
        int _dummy
    cdef struct dxSpace:
        int _dummy
    cdef struct dxBody:
        int _dummy
    cdef struct dxGeom:
        int _dummy
    cdef struct dxJoint:
        int _dummy
    cdef struct dxJointGroup:
        int _dummy
    cdef struct dxTriMeshData:
        int _dummy
    cdef struct dxHeightfieldData:
        int _dummy

    # Types
    ctypedef dxWorld* dWorldID
    ctypedef dxSpace* dSpaceID
    ctypedef dxBody* dBodyID
    ctypedef dxGeom* dGeomID
    ctypedef dxJoint* dJointID
    ctypedef dxJointGroup* dJointGroupID
    ctypedef dxTriMeshData* dTriMeshDataID
    ctypedef dxHeightfieldData* dHeightfieldDataID
    ctypedef dReal dVector3[4]
    ctypedef dReal dVector4[4]
    ctypedef dReal dMatrix3[4*3]
    ctypedef dReal dMatrix4[4*4]
    ctypedef dReal dMatrix6[8*6]
    ctypedef dReal dQuaternion[4]

    cdef extern dReal dInfinity
    cdef extern int dAMotorUser
    cdef extern int dAMotorEuler

    ctypedef struct dMass:
        dReal    mass
        dVector4 c
        dMatrix3 I

    ctypedef struct dJointFeedback:
        dVector3 f1
        dVector3 t1
        dVector3 f2
        dVector3 t2

    ctypedef void dNearCallback(void* data, dGeomID o1, dGeomID o2)
    ctypedef dReal dHeightfieldGetHeight( void* p_user_data, int x, int z )

    ctypedef void dGetAABBFn (dGeomID, dReal aabb[6])
    ctypedef int dColliderFn (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip)
    ctypedef dColliderFn * dGetColliderFnFn (int num)
    ctypedef void dGeomDtorFn (dGeomID o)
    ctypedef int dAABBTestFn (dGeomID o1, dGeomID o2, dReal aabb[6])

    ctypedef struct dSurfaceParameters:
        int mode
        dReal mu

        dReal mu2
        dReal bounce
        dReal bounce_vel
        dReal soft_erp
        dReal soft_cfm
        dReal motion1,motion2
        dReal slip1,slip2

    ctypedef struct dContactGeom:
        dVector3 pos
        dVector3 normal
        dReal depth
        dGeomID g1,g2

    ctypedef struct dContact:
        dSurfaceParameters surface
        dContactGeom geom
        dVector3 fdir1

    ctypedef struct dGeomClass:
        int bytes
        dGetColliderFnFn *collider
        dGetAABBFn *aabb
        dAABBTestFn *aabb_test
        dGeomDtorFn *dtor

    ctypedef struct dWorldStepReserveInfo:
        unsigned struct_size
        float reserve_factor
        unsigned reserve_minimum

    ctypedef struct dWorldStepMemoryFunctionsInfo:
        pass
#        unsigned struct_size
#        void *(*alloc_block)(dsizeint block_size)
#        void *(*shrink_block)(void *block_pointer, dsizeint block_current_size, dsizeint block_smaller_size)
#        void (*free_block)(void *block_pointer, dsizeint block_current_size)


    # ODE
    void dInitODE()
    void dCloseODE()


    # World
    dWorldID dWorldCreate()
    void dWorldDestroy (dWorldID)

    #not actually present in the manual
    void dWorldSetData (dWorldID world, void* data)
    void* dWorldGetData (dWorldID world)

    void dWorldSetGravity (dWorldID, dReal x, dReal y, dReal z)
    void dWorldGetGravity (dWorldID, dVector3 gravity)

    void dWorldSetERP (dWorldID, dReal erp)
    dReal dWorldGetERP (dWorldID)

    void dWorldSetCFM (dWorldID, dReal cfm)
    dReal dWorldGetCFM (dWorldID)

    # not present in the manual
    void dWorldSetStepIslandsProcessingMaxThreadCount(dWorldID w, unsigned count)
    unsigned dWorldGetStepIslandsProcessingMaxThreadCount(dWorldID w)

    # not present in the manual
    int dWorldUseSharedWorkingMemory(dWorldID w, dWorldID from_world)
    void dWorldCleanupWorkingMemory(dWorldID w)

    # not present in the manual
    int dWorldSetStepMemoryReservationPolicy(dWorldID w, const dWorldStepReserveInfo *policyinfo)

    # not present in the manual
    int dWorldSetStepMemoryManager(dWorldID w, const dWorldStepMemoryFunctionsInfo *memfuncs)

    # not present in the manual
    # TODO: no real idea where dThreadingImplementationID should come from
    #void dWorldSetStepThreadingImplementation(dWorldID w, const dThreadingFunctionsInfo *functions_info, dThreadingImplementationID threading_impl)

    void dWorldStep (dWorldID, dReal stepsize)
    void dWorldQuickStep (dWorldID, dReal stepsize)

    void dWorldImpulseToForce (dWorldID, dReal stepsize, dReal ix, dReal iy,
        dReal iz, dVector3 force)

    void dWorldSetQuickStepNumIterations (dWorldID, int num)
    int dWorldGetQuickStepNumIterations (dWorldID)

    void dWorldSetQuickStepW (dWorldID, dReal over_relaxation)
    dReal dWorldGetQuickStepW (dWorldID)

    void dWorldSetContactMaxCorrectingVel (dWorldID, dReal vel)
    dReal dWorldGetContactMaxCorrectingVel (dWorldID)

    void dWorldSetContactSurfaceLayer (dWorldID, dReal depth)
    dReal dWorldGetContactSurfaceLayer (dWorldID)

    dReal dWorldGetAutoDisableLinearThreshold (dWorldID)
    void dWorldSetAutoDisableLinearThreshold (dWorldID, dReal linear_threshold)

    dReal dWorldGetAutoDisableAngularThreshold (dWorldID)
    void dWorldSetAutoDisableAngularThreshold (dWorldID, dReal angular_threshold)

    # not present in the manual
    int dWorldGetAutoDisableAverageSamplesCount (dWorldID)
    void dWorldSetAutoDisableAverageSamplesCount (dWorldID,
        unsigned int average_samples_count)

    int dWorldGetAutoDisableSteps (dWorldID)
    void dWorldSetAutoDisableSteps (dWorldID, int steps)

    dReal dWorldGetAutoDisableTime (dWorldID)
    void dWorldSetAutoDisableTime (dWorldID, dReal time)

    int dWorldGetAutoDisableFlag (dWorldID)
    void dWorldSetAutoDisableFlag (dWorldID, int do_auto_disable)

    dReal dWorldGetLinearDampingThreshold (dWorldID w)
    void dWorldSetLinearDampingThreshold(dWorldID w, dReal threshold)

    dReal dWorldGetAngularDampingThreshold (dWorldID w)
    void dWorldSetAngularDampingThreshold(dWorldID w, dReal threshold)

    dReal dWorldGetLinearDamping (dWorldID)
    void dWorldSetLinearDamping (dWorldID, dReal scale)

    dReal dWorldGetAngularDamping (dWorldID)
    void dWorldSetAngularDamping (dWorldID, dReal scale)

    void dWorldSetDamping(dWorldID w, dReal linear_scale, dReal angular_scale)

    dReal dWorldGetMaxAngularSpeed (dWorldID w)
    void dWorldSetMaxAngularSpeed (dWorldID w, dReal max_speed)


    # Body
    dReal dBodyGetAutoDisableLinearThreshold (dBodyID)
    void  dBodySetAutoDisableLinearThreshold (dBodyID, dReal linear_average_threshold)

    dReal dBodyGetAutoDisableAngularThreshold (dBodyID)
    void  dBodySetAutoDisableAngularThreshold (dBodyID, dReal angular_average_threshold)

    int dBodyGetAutoDisableAverageSamplesCount (dBodyID)
    void dBodySetAutoDisableAverageSamplesCount (dBodyID, unsigned int average_samples_count)

    int dBodyGetAutoDisableSteps (dBodyID)
    void dBodySetAutoDisableSteps (dBodyID, int steps)

    dReal dBodyGetAutoDisableTime (dBodyID)
    void  dBodySetAutoDisableTime (dBodyID, dReal time)

    int dBodyGetAutoDisableFlag (dBodyID)
    void dBodySetAutoDisableFlag (dBodyID, int do_auto_disable)

    void  dBodySetAutoDisableDefaults (dBodyID)

    dWorldID dBodyGetWorld (dBodyID)

    dBodyID dBodyCreate (dWorldID)
    void dBodyDestroy (dBodyID)

    void  dBodySetData (dBodyID, void *data)
    void *dBodyGetData (dBodyID)

    void dBodySetPosition   (dBodyID, dReal x, dReal y, dReal z)
    void dBodySetRotation   (dBodyID, dMatrix3 R)
    void dBodySetQuaternion (dBodyID, dQuaternion q)
    void dBodySetLinearVel  (dBodyID, dReal x, dReal y, dReal z)
    void dBodySetAngularVel (dBodyID, dReal x, dReal y, dReal z)

    dReal * dBodyGetPosition   (dBodyID)
    void dBodyCopyPosition (dBodyID body, dVector3 pos)
    dReal * dBodyGetRotation   (dBodyID)
    void dBodyCopyRotation (dBodyID, dMatrix3 R)
    dReal * dBodyGetQuaternion (dBodyID)
    void dBodyCopyQuaternion(dBodyID body, dQuaternion quat)
    dReal * dBodyGetLinearVel  (dBodyID)
    dReal * dBodyGetAngularVel (dBodyID)

    void dBodySetMass (dBodyID, dMass *mass)
    void dBodyGetMass (dBodyID, dMass *mass)

    void dBodyAddForce            (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddTorque           (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddRelForce         (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddRelTorque        (dBodyID, dReal fx, dReal fy, dReal fz)
    void dBodyAddForceAtPos       (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)
    void dBodyAddForceAtRelPos    (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)
    void dBodyAddRelForceAtPos    (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)
    void dBodyAddRelForceAtRelPos (dBodyID, dReal fx, dReal fy, dReal fz, dReal px, dReal py, dReal pz)

    dReal * dBodyGetForce   (dBodyID)
    dReal * dBodyGetTorque  (dBodyID)

    void dBodySetForce(dBodyID, dReal x, dReal y, dReal z)
    void dBodySetTorque(dBodyID, dReal x, dReal y, dReal z)

    void dBodyGetRelPointPos    (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)
    void dBodyGetRelPointVel    (dBodyID, dReal px, dReal py, dReal pz, dVector3 result)
    void dBodyGetPointVel    (dBodyID, dReal px, dReal py, dReal pz,
                              dVector3 result)
    void dBodyGetPosRelPoint (dBodyID, dReal px, dReal py, dReal pz,
                              dVector3 result)

    void dBodyVectorToWorld   (dBodyID, dReal px, dReal py, dReal pz,
                               dVector3 result)
    void dBodyVectorFromWorld (dBodyID, dReal px, dReal py, dReal pz,
                               dVector3 result)

    void dBodySetFiniteRotationMode (dBodyID, int mode)
    void dBodySetFiniteRotationAxis (dBodyID, dReal x, dReal y, dReal z)

    int dBodyGetFiniteRotationMode (dBodyID)
    void dBodyGetFiniteRotationAxis (dBodyID, dVector3 result)

    int dBodyGetNumJoints (dBodyID b)
    dJointID dBodyGetJoint (dBodyID, int index)

    void dBodySetDynamic (dBodyID)
    void dBodySetKinematic (dBodyID)
    int dBodyIsKinematic (dBodyID)

    void dBodyEnable (dBodyID)
    void dBodyDisable (dBodyID)
    int dBodyIsEnabled (dBodyID)

    void dBodySetGravityMode (dBodyID b, int mode)
    int dBodyGetGravityMode (dBodyID b)

    void dBodySetMovedCallback(dBodyID b, void (*callback)(dBodyID))

    dGeomID dBodyGetFirstGeom (dBodyID b)
    dGeomID dBodyGetNextGeom (dGeomID g)

    void dBodySetDampingDefaults(dBodyID b)

    dReal dBodyGetLinearDamping (dBodyID b)
    void dBodySetLinearDamping(dBodyID b, dReal scale)

    dReal dBodyGetAngularDamping (dBodyID b)
    void dBodySetAngularDamping(dBodyID b, dReal scale)

    void dBodySetDamping(dBodyID b, dReal linear_scale, dReal angular_scale)

    dReal dBodyGetLinearDampingThreshold (dBodyID b)
    void dBodySetLinearDampingThreshold(dBodyID b, dReal threshold)

    dReal dBodyGetAngularDampingThreshold (dBodyID b)
    void dBodySetAngularDampingThreshold(dBodyID b, dReal threshold)

    dReal dBodyGetMaxAngularSpeed (dBodyID b)
    void dBodySetMaxAngularSpeed(dBodyID b, dReal max_speed)

    # not present in the manual
    int dBodyGetGyroscopicMode(dBodyID b);
    void dBodySetGyroscopicMode(dBodyID b, int enabled);


    # Joints
    dJointID dJointCreateBall (dWorldID, dJointGroupID)
    dJointID dJointCreateHinge (dWorldID, dJointGroupID)
    dJointID dJointCreateSlider (dWorldID, dJointGroupID)
    dJointID dJointCreateContact (dWorldID, dJointGroupID, dContact *)
    dJointID dJointCreateHinge2 (dWorldID, dJointGroupID)
    dJointID dJointCreateUniversal (dWorldID, dJointGroupID)
    dJointID dJointCreatePR (dWorldID, dJointGroupID)
    dJointID dJointCreatePU (dWorldID, dJointGroupID)
    dJointID dJointCreatePiston (dWorldID, dJointGroupID)
    dJointID dJointCreateFixed (dWorldID, dJointGroupID)
    dJointID dJointCreateNull (dWorldID, dJointGroupID)
    dJointID dJointCreateAMotor (dWorldID, dJointGroupID)
    dJointID dJointCreateLMotor (dWorldID, dJointGroupID)
    dJointID dJointCreatePlane2D (dWorldID, dJointGroupID)
    dJointID dJointCreateDBall (dWorldID, dJointGroupID)
    dJointID dJointCreateDHinge (dWorldID, dJointGroupID)
    dJointID dJointCreateTransmission (dWorldID, dJointGroupID)

    void dJointDestroy (dJointID)

    dJointGroupID dJointGroupCreate (int max_size)
    void dJointGroupDestroy (dJointGroupID)
    void dJointGroupEmpty (dJointGroupID)

    int dJointGetNumBodies(dJointID)

    void dJointAttach (dJointID, dBodyID body1, dBodyID body2)

    void dJointEnable (dJointID)
    void dJointDisable (dJointID)
    int dJointIsEnabled (dJointID)

    void dJointSetData (dJointID, void *data)
    void *dJointGetData (dJointID)

    int dJointGetType (dJointID)

    dBodyID dJointGetBody (dJointID, int index)

    void dJointSetFeedback (dJointID, dJointFeedback *)
    dJointFeedback *dJointGetFeedback (dJointID)

    void dJointSetBallAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetBallAnchor2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetBallParam (dJointID, int parameter, dReal value)

    void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHingeAnchorDelta (dJointID, dReal x, dReal y, dReal z, dReal ax,
        dReal ay, dReal az)
    void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHingeAxisOffset (dJointID j, dReal x, dReal y, dReal z, dReal angle)
    void dJointSetHingeParam (dJointID, int parameter, dReal value)
    void dJointAddHingeTorque(dJointID joint, dReal torque)

    void dJointSetSliderAxis (dJointID, dReal x, dReal y, dReal z)
    void dJointSetSliderAxisDelta (dJointID, dReal x, dReal y, dReal z, dReal ax,
        dReal ay, dReal az)
    void dJointSetSliderParam (dJointID, int parameter, dReal value)
    void dJointAddSliderForce(dJointID joint, dReal force)

    void dJointSetHinge2Anchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHinge2Axes (dJointID j, const dReal *axis1, const dReal *axis2)
    # ODE_API_DEPRECATED
    void dJointSetHinge2Axis1 (dJointID, dReal x, dReal y, dReal z)
    # ODE_API_DEPRECATED
    void dJointSetHinge2Axis2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetHinge2Param (dJointID, int parameter, dReal value)
    void dJointAddHinge2Torques(dJointID joint, dReal torque1, dReal torque2)

    void dJointSetUniversalAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetUniversalAxis1 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetUniversalAxis1Offset (dJointID, dReal x, dReal y, dReal z,
        dReal offset1, dReal offset2)
    void dJointSetUniversalAxis2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetUniversalAxis2Offset (dJointID, dReal x, dReal y, dReal z,
        dReal offset1, dReal offset2)
    void dJointSetUniversalParam (dJointID, int parameter, dReal value)
    void dJointAddUniversalTorques(dJointID joint, dReal torque1, dReal torque2)

    void dJointSetPRAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPRAxis1 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPRAxis2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPRParam (dJointID, int parameter, dReal value)
    void dJointAddPRTorque (dJointID j, dReal torque)

    void dJointSetPUAnchor (dJointID, dReal x, dReal y, dReal z)
    # ODE_API_DEPRECATED - included for completeness' sake
    #void dJointSetPUAnchorDelta (dJointID, dReal x, dReal y, dReal z, dReal dx,
    #    dReal dy, dReal dz)
    void dJointSetPUAnchorOffset (dJointID, dReal x, dReal y, dReal z, dReal dx,
        dReal dy, dReal dz)
    void dJointSetPUAxis1 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPUAxis2 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPUAxis3 (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPUAxisP (dJointID id, dReal x, dReal y, dReal z)
    void dJointSetPUParam (dJointID, int parameter, dReal value)
    void dJointAddPUTorque (dJointID j, dReal torque)

    void dJointSetPistonAnchor (dJointID, dReal x, dReal y, dReal z)
    void dJointSetPistonAnchorOffset(dJointID j, dReal x, dReal y, dReal z,
        dReal dx, dReal dy, dReal dz)
    void dJointSetPistonAxis (dJointID, dReal x, dReal y, dReal z)
    # ODE_API_DEPRECATED - included for completeness' sake
    #void dJointSetPistonAxisDelta (dJointID j, dReal x, dReal y, dReal z,
    #    dReal ax, dReal ay, dReal az)
    void dJointSetPistonParam (dJointID, int parameter, dReal value)
    void dJointAddPistonForce (dJointID joint, dReal force)

    void dJointSetFixed (dJointID)
    void dJointSetFixedParam (dJointID, int parameter, dReal value)

    void dJointSetAMotorNumAxes (dJointID, int num)
    void dJointSetAMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z)
    void dJointSetAMotorAngle (dJointID, int anum, dReal angle)
    void dJointSetAMotorParam (dJointID, int parameter, dReal value)
    void dJointSetAMotorMode (dJointID, int mode)
    void dJointAddAMotorTorques (dJointID, dReal torque1, dReal torque2, dReal torque3)

    void dJointSetLMotorNumAxes (dJointID, int num)
    void dJointSetLMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z)
    void dJointSetLMotorParam (dJointID, int parameter, dReal value)

    void dJointSetPlane2DXParam (dJointID, int parameter, dReal value)
    void dJointSetPlane2DYParam (dJointID, int parameter, dReal value)
    void dJointSetPlane2DAngleParam (dJointID, int parameter, dReal value)

    void dJointGetBallAnchor (dJointID, dVector3 result)
    void dJointGetBallAnchor2 (dJointID, dVector3 result)
    dReal dJointGetBallParam (dJointID, int parameter)

    void dJointGetHingeAnchor (dJointID, dVector3 result)
    void dJointGetHingeAnchor2 (dJointID, dVector3 result)
    void dJointGetHingeAxis (dJointID, dVector3 result)
    dReal dJointGetHingeParam (dJointID, int parameter)
    dReal dJointGetHingeAngle (dJointID)
    dReal dJointGetHingeAngleRate (dJointID)

    dReal dJointGetSliderPosition (dJointID)
    dReal dJointGetSliderPositionRate (dJointID)
    void dJointGetSliderAxis (dJointID, dVector3 result)
    dReal dJointGetSliderParam (dJointID, int parameter)

    void dJointGetHinge2Anchor (dJointID, dVector3 result)
    void dJointGetHinge2Anchor2 (dJointID, dVector3 result)
    void dJointGetHinge2Axis1 (dJointID, dVector3 result)
    void dJointGetHinge2Axis2 (dJointID, dVector3 result)
    dReal dJointGetHinge2Param (dJointID, int parameter)
    dReal dJointGetHinge2Angle1 (dJointID)
    dReal dJointGetHinge2Angle2 (dJointID);
    dReal dJointGetHinge2Angle1Rate (dJointID)
    dReal dJointGetHinge2Angle2Rate (dJointID)

    void dJointGetUniversalAnchor (dJointID, dVector3 result)
    void dJointGetUniversalAnchor2 (dJointID, dVector3 result)
    void dJointGetUniversalAxis1 (dJointID, dVector3 result)
    void dJointGetUniversalAxis2 (dJointID, dVector3 result)
    dReal dJointGetUniversalParam (dJointID, int parameter)
    void dJointGetUniversalAngles (dJointID, dReal *angle1, dReal *angle2)
    dReal dJointGetUniversalAngle1 (dJointID)
    dReal dJointGetUniversalAngle2 (dJointID)
    dReal dJointGetUniversalAngle1Rate (dJointID)
    dReal dJointGetUniversalAngle2Rate (dJointID)

    dReal dJointGetPRPosition (dJointID)
    dReal dJointGetPRPositionRate (dJointID)
    dReal dJointGetPRAngle (dJointID)
    dReal dJointGetPRAngleRate (dJointID)
    void dJointGetPRAxis1 (dJointID, dVector3 result)
    void dJointGetPRAxis2 (dJointID, dVector3 result)
    dReal dJointGetPRParam (dJointID, int parameter)

    void dJointGetPUAnchor (dJointID, dVector3 result)
    dReal dJointGetPUPosition (dJointID)
    dReal dJointGetPUPositionRate (dJointID)
    void dJointGetPUAxis1 (dJointID, dVector3 result)
    void dJointGetPUAxis2 (dJointID, dVector3 result)
    void dJointGetPUAxis3 (dJointID, dVector3 result)
    void dJointGetPUAxisP (dJointID id, dVector3 result)
    void dJointGetPUAngles (dJointID, dReal *angle1, dReal *angle2)
    dReal dJointGetPUAngle1 (dJointID)
    dReal dJointGetPUAngle1Rate (dJointID)
    dReal dJointGetPUAngle2 (dJointID)
    dReal dJointGetPUAngle2Rate (dJointID)
    dReal dJointGetPUParam (dJointID, int parameter)

    dReal dJointGetPistonPosition (dJointID)
    dReal dJointGetPistonPositionRate (dJointID)
    dReal dJointGetPistonAngle (dJointID)
    dReal dJointGetPistonAngleRate (dJointID)
    void dJointGetPistonAnchor (dJointID, dVector3 result)
    void dJointGetPistonAnchor2 (dJointID, dVector3 result)
    void dJointGetPistonAxis (dJointID, dVector3 result)
    dReal dJointGetPistonParam (dJointID, int parameter)

    int dJointGetAMotorNumAxes (dJointID)
    void dJointGetAMotorAxis (dJointID, int anum, dVector3 result)
    int dJointGetAMotorAxisRel (dJointID, int anum)
    dReal dJointGetAMotorAngle (dJointID, int anum)
    dReal dJointGetAMotorAngleRate (dJointID, int anum)
    dReal dJointGetAMotorParam (dJointID, int parameter)
    int dJointGetAMotorMode (dJointID)

    int dJointGetLMotorNumAxes (dJointID)
    void dJointGetLMotorAxis (dJointID, int anum, dVector3 result)
    dReal dJointGetLMotorParam (dJointID, int parameter)

    dReal dJointGetFixedParam (dJointID, int parameter)

    void dJointGetTransmissionContactPoint1(dJointID, dVector3 result)
    void dJointGetTransmissionContactPoint2(dJointID, dVector3 result)
    void dJointSetTransmissionAxis1(dJointID, dReal x, dReal y, dReal z)
    void dJointGetTransmissionAxis1(dJointID, dVector3 result)
    void dJointSetTransmissionAxis2(dJointID, dReal x, dReal y, dReal z)
    void dJointGetTransmissionAxis2(dJointID, dVector3 result)
    void dJointSetTransmissionAnchor1(dJointID, dReal x, dReal y, dReal z)
    void dJointGetTransmissionAnchor1(dJointID, dVector3 result)
    void dJointSetTransmissionAnchor2(dJointID, dReal x, dReal y, dReal z)
    void dJointGetTransmissionAnchor2(dJointID, dVector3 result)
    void dJointSetTransmissionParam(dJointID, int parameter, dReal value)
    dReal dJointGetTransmissionParam(dJointID, int parameter)
    void dJointSetTransmissionMode( dJointID j, int mode )
    int dJointGetTransmissionMode( dJointID j )
    void dJointSetTransmissionRatio( dJointID j, dReal ratio )
    dReal dJointGetTransmissionRatio( dJointID j )
    void dJointSetTransmissionAxis( dJointID j, dReal x, dReal y, dReal z )
    void dJointGetTransmissionAxis( dJointID j, dVector3 result )
    dReal dJointGetTransmissionAngle1( dJointID j )
    dReal dJointGetTransmissionAngle2( dJointID j )
    dReal dJointGetTransmissionRadius1( dJointID j )
    dReal dJointGetTransmissionRadius2( dJointID j )
    void dJointSetTransmissionRadius1( dJointID j, dReal radius )
    void dJointSetTransmissionRadius2( dJointID j, dReal radius )
    dReal dJointGetTransmissionBacklash( dJointID j )
    void dJointSetTransmissionBacklash( dJointID j, dReal backlash )

    void dJointSetDBallAnchor1(dJointID, dReal x, dReal y, dReal z)
    void dJointSetDBallAnchor2(dJointID, dReal x, dReal y, dReal z)
    void dJointGetDBallAnchor1(dJointID, dVector3 result)
    void dJointGetDBallAnchor2(dJointID, dVector3 result)
    dReal dJointGetDBallDistance(dJointID)
    void dJointSetDBallDistance(dJointID, dReal dist)
    void dJointSetDBallParam(dJointID, int parameter, dReal value)
    dReal dJointGetDBallParam(dJointID, int parameter)

    void dJointSetDHingeAxis(dJointID, dReal x, dReal y, dReal z)
    void dJointGetDHingeAxis(dJointID, dVector3 result)
    void dJointSetDHingeAnchor1(dJointID, dReal x, dReal y, dReal z)
    void dJointSetDHingeAnchor2(dJointID, dReal x, dReal y, dReal z)
    void dJointGetDHingeAnchor1(dJointID, dVector3 result)
    void dJointGetDHingeAnchor2(dJointID, dVector3 result)
    dReal dJointGetDHingeDistance(dJointID)
    void dJointSetDHingeParam(dJointID, int parameter, dReal value)
    dReal dJointGetDHingeParam(dJointID, int parameter)

    dJointID dConnectingJoint (dBodyID, dBodyID)
    int dConnectingJointList (dBodyID, dBodyID, dJointID*)
    int dAreConnected (dBodyID, dBodyID)
    int dAreConnectedExcluding (dBodyID body1, dBodyID body2, int joint_type)


    # Mass
    void dMassSetZero (dMass *)
    void dMassSetParameters (dMass *, dReal themass,
             dReal cgx, dReal cgy, dReal cgz,
             dReal I11, dReal I22, dReal I33,
             dReal I12, dReal I13, dReal I23)
    void dMassSetSphere (dMass *, dReal density, dReal radius)
    void dMassSetSphereTotal (dMass *, dReal total_mass, dReal radius)
    void dMassSetCappedCylinder (dMass *, dReal density, int direction,
                 dReal a, dReal b)
    void dMassSetCappedCylinderTotal (dMass *, dReal total_mass, int direction,
                 dReal a, dReal b)
    void dMassSetCylinder (dMass *, dReal density, int direction,
          dReal radius, dReal length)
    void dMassSetCylinderTotal (dMass *, dReal total_mass, int direction,
          dReal radius, dReal length)
    void dMassSetBox (dMass *, dReal density,
          dReal lx, dReal ly, dReal lz)
    void dMassSetBoxTotal (dMass *, dReal total_mass,
          dReal lx, dReal ly, dReal lz)
    void dMassAdjust (dMass *, dReal newmass)
    void dMassTranslate (dMass *, dReal x, dReal y, dReal z)
    void dMassRotate (dMass *, dMatrix3 R)
    void dMassAdd (dMass *a, dMass *b)

    # Space
    dSpaceID dSimpleSpaceCreate(dSpaceID space)
    dSpaceID dHashSpaceCreate(dSpaceID space)
    dSpaceID dQuadTreeSpaceCreate (dSpaceID space, dVector3 Center, dVector3 Extents, int Depth)
    # not present in the manual
    #define dSAP_AXES_XYZ  ((0)|(1<<2)|(2<<4)) - 0 | 100 | 110000 = 110100 = 4 + 16 + 32 = 52
    #define dSAP_AXES_XZY  ((0)|(2<<2)|(1<<4)) - 0 | 1000 | 10000 = 11000 = 8 + 16 = 24
    #define dSAP_AXES_YXZ  ((1)|(0<<2)|(2<<4)) - 1 | 000 | 100000 = 100001 = 1 + 32 = 33
    #define dSAP_AXES_YZX  ((1)|(2<<2)|(0<<4)) - 1 | 1000 | 00000 = 01001 = 1 + 8 = 9
    #define dSAP_AXES_ZXY  ((2)|(0<<2)|(1<<4)) - 10 | 000 | 10000 = 10010 = 2 + 16 = 18
    #define dSAP_AXES_ZYX  ((2)|(1<<2)|(0<<4)) - 10 | 100 | 00000 = 00110 = 2 + 4 = 6
    dSpaceID dSweepAndPruneSpaceCreate(dSpaceID space, int axisorder)

    void dSpaceDestroy (dSpaceID)

    void dHashSpaceSetLevels (dSpaceID space, int minlevel, int maxlevel)
    void dHashSpaceGetLevels (dSpaceID space, int *minlevel, int *maxlevel)

    void dSpaceSetCleanup (dSpaceID space, int mode)
    int dSpaceGetCleanup (dSpaceID space)

    void dSpaceSetSublevel (dSpaceID space, int sublevel)
    int dSpaceGetSublevel (dSpaceID space)

    # not present in the manual
    void dSpaceSetManualCleanup (dSpaceID space, int mode)
    int dSpaceGetManualCleanup (dSpaceID space)

    void dSpaceAdd (dSpaceID, dGeomID)
    void dSpaceRemove (dSpaceID, dGeomID)

    int dSpaceQuery (dSpaceID, dGeomID)

    # not present in the manual
    void dSpaceClean (dSpaceID)

    int dSpaceGetNumGeoms (dSpaceID)
    dGeomID dSpaceGetGeom (dSpaceID, int i)

    int dSpaceGetClass(dSpaceID space)

    # Collision Detection
    int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip)

    void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback)

    void dSpaceCollide2 (dGeomID o1, dGeomID o2, void *data, dNearCallback *callback)

    # Geom
    void dGeomDestroy (dGeomID)

    void dGeomSetData (dGeomID, void *)
    void *dGeomGetData (dGeomID)

    void dGeomSetBody (dGeomID, dBodyID)
    dBodyID dGeomGetBody (dGeomID)

    void dGeomSetPosition (dGeomID, dReal x, dReal y, dReal z)
    void dGeomSetRotation (dGeomID, dMatrix3 R)
    void dGeomSetQuaternion (dGeomID, dQuaternion)

    dReal * dGeomGetPosition (dGeomID)
    dReal * dGeomGetRotation (dGeomID)
    void dGeomGetQuaternion (dGeomID, dQuaternion result)

    # not present in the manual
    void dGeomCopyPosition (dGeomID geom, dVector3 pos)
    void dGeomCopyRotation(dGeomID geom, dMatrix3 R)

    void dGeomGetAABB (dGeomID, dReal aabb[6])

    int dGeomIsSpace (dGeomID)
    dSpaceID dGeomGetSpace (dGeomID)
    int dGeomGetClass (dGeomID)

    void dGeomSetCategoryBits(dGeomID, unsigned long bits)
    void dGeomSetCollideBits(dGeomID, unsigned long bits)
    unsigned long dGeomGetCategoryBits(dGeomID)
    unsigned long dGeomGetCollideBits(dGeomID)

    void dGeomEnable (dGeomID)
    void dGeomDisable (dGeomID)
    int dGeomIsEnabled (dGeomID)

    # not present in the manual
    int dGeomLowLevelControl (dGeomID geom, int controlClass, int controlCode,
        void *dataValue, int *dataSize)

    # not present in the manual
    void dGeomGetRelPointPos (dGeomID geom, dReal px, dReal py, dReal pz,
        dVector3 result)
    void dGeomGetPosRelPoint (dGeomID geom, dReal px, dReal py, dReal pz,
        dVector3 result)

    # not present in the manual
    void dGeomVectorToWorld (dGeomID geom, dReal px, dReal py, dReal pz,
        dVector3 result)
    void dGeomVectorFromWorld (dGeomID geom, dReal px, dReal py, dReal pz,
        dVector3 result)

    void dGeomSetOffsetPosition (dGeomID geom, dReal x, dReal y, dReal z)
    void dGeomSetOffsetRotation (dGeomID geom, const dMatrix3 R)
    void dGeomSetOffsetQuaternion (dGeomID geom, const dQuaternion Q)

    void dGeomSetOffsetWorldPosition (dGeomID geom, dReal x, dReal y, dReal z)
    void dGeomSetOffsetWorldRotation (dGeomID geom, const dMatrix3 R)
    void dGeomSetOffsetWorldQuaternion (dGeomID geom, const dQuaternion)

    void dGeomClearOffset(dGeomID geom)
    # not present in the manual
    int dGeomIsOffset(dGeomID geom)

    const dReal * dGeomGetOffsetPosition (dGeomID geom)
    const dReal * dGeomGetOffsetRotation (dGeomID geom)
    void dGeomGetOffsetQuaternion (dGeomID geom, dQuaternion result)

    # not present in the manual
    void dGeomCopyOffsetPosition (dGeomID geom, dVector3 pos)
    void dGeomCopyOffsetRotation (dGeomID geom, dMatrix3 R)

        # Sphere
    dGeomID dCreateSphere (dSpaceID space, dReal radius)
    void dGeomSphereSetRadius (dGeomID sphere, dReal radius)
    dReal dGeomSphereGetRadius (dGeomID sphere)
    dReal dGeomSpherePointDepth (dGeomID sphere, dReal x, dReal y, dReal z)

        # Convex
    dGeomID dCreateConvex (dSpaceID space, const dReal *_planes,
        unsigned int _planecount, const dReal *_points, unsigned int _pointcount,
        const unsigned int *_polygons)
    void dGeomSetConvex (dGeomID g, const dReal *_planes, unsigned int _count,
        const dReal *_points, unsigned int _pointcount, const unsigned int *_polygons)

        # Box
    dGeomID dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)
    void dGeomBoxSetLengths (dGeomID box, dReal lx, dReal ly, dReal lz)
    void  dGeomBoxGetLengths (dGeomID box, dVector3 result)
    dReal dGeomBoxPointDepth (dGeomID box, dReal x, dReal y, dReal z)

        # Plane
    dGeomID dCreatePlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d)
    void dGeomPlaneSetParams (dGeomID plane, dReal a, dReal b, dReal c, dReal d)
    void  dGeomPlaneGetParams (dGeomID plane, dVector4 result)
    dReal dGeomPlanePointDepth (dGeomID plane, dReal x, dReal y, dReal z)

        # Capsule
    dGeomID dCreateCapsule (dSpaceID space, dReal radius, dReal length)
    void dGeomCapsuleSetParams (dGeomID ccylinder, dReal radius, dReal length)
    void  dGeomCapsuleGetParams (dGeomID ccylinder, dReal *radius, dReal *length)
    dReal dGeomCapsulePointDepth (dGeomID ccylinder, dReal x, dReal y, dReal z)

        # Cylinder
    dGeomID dCreateCylinder (dSpaceID space, dReal radius, dReal length)
    void dGeomCylinderSetParams (dGeomID ccylinder, dReal radius, dReal length)
    void  dGeomCylinderGetParams (dGeomID ccylinder, dReal *radius, dReal *length)

        # Ray
    dGeomID dCreateRay (dSpaceID space, dReal length)
    void dGeomRaySetLength (dGeomID ray, dReal length)
    dReal dGeomRayGetLength (dGeomID ray)
    void dGeomRaySet (dGeomID ray, dReal px, dReal py, dReal pz, dReal dx,
        dReal dy, dReal dz)
    void dGeomRayGet (dGeomID ray, dVector3 start, dVector3 dir)
    void dGeomRaySetFirstContact (dGeomID g, int firstContact)
    int dGeomRayGetFirstContact (dGeomID g)
    void dGeomRaySetBackfaceCull (dGeomID g, int backfaceCull)
    int dGeomRayGetBackfaceCull (dGeomID g)
    void dGeomRaySetClosestHit (dGeomID g, int closestHit)
    int dGeomRayGetClosestHit (dGeomID g)

        # Heightfield
    dGeomID dCreateHeightfield (dSpaceID space, dHeightfieldDataID data,
        int bPlaceable)
    dHeightfieldDataID dGeomHeightfieldDataCreate()
    void dGeomHeightfieldDataDestroy(dHeightfieldDataID g)
    void dGeomHeightfieldDataBuildCallback(dHeightfieldDataID d, void* pUserData,
        dHeightfieldGetHeight* pCallback, dReal width, dReal depth,
        int widthSamples, int depthSamples, dReal scale, dReal offset,
        dReal thickness, int bWrap)
    void dGeomHeightfieldDataBuildByte( dHeightfieldDataID d,
        const unsigned char* pHeightData, int bCopyHeightData, dReal width,
        dReal depth, int widthSamples, int depthSamples, dReal scale,
        dReal offset, dReal thickness,	int bWrap )
    void dGeomHeightfieldDataBuildShort( dHeightfieldDataID d,
        const short* pHeightData, int bCopyHeightData,
        dReal width, dReal depth, int widthSamples, int depthSamples,
        dReal scale, dReal offset, dReal thickness, int bWrap )
    void dGeomHeightfieldDataBuildSingle( dHeightfieldDataID d,
        const float* pHeightData, int bCopyHeightData,
        dReal width, dReal depth, int widthSamples, int depthSamples,
        dReal scale, dReal offset, dReal thickness, int bWrap )
    void dGeomHeightfieldDataBuildDouble( dHeightfieldDataID d,
        const double* pHeightData, int bCopyHeightData,
        dReal width, dReal depth, int widthSamples, int depthSamples,
        dReal scale, dReal offset, dReal thickness, int bWrap )
    void dGeomHeightfieldDataSetBounds( dHeightfieldDataID d,
        dReal minHeight, dReal maxHeight )
    void dGeomHeightfieldSetHeightfieldData( dGeomID g, dHeightfieldDataID d )
    dHeightfieldDataID dGeomHeightfieldGetHeightfieldData( dGeomID g )

        # Utility
    void dClosestLineSegmentPoints (const dVector3 a1, const dVector3 a2,
        const dVector3 b1, const dVector3 b2, dVector3 cp1, dVector3 cp2)
    int dBoxTouchesBox (const dVector3 _p1, const dMatrix3 R1,
        const dVector3 side1, const dVector3 _p2,
        const dMatrix3 R2, const dVector3 side2)
    int dBoxBox (const dVector3 p1, const dMatrix3 R1,
        const dVector3 side1, const dVector3 p2,
        const dMatrix3 R2, const dVector3 side2,
        dVector3 normal, dReal *depth, int *return_code,
        int flags, dContactGeom *contact, int skip)
    void dInfiniteAABB (dGeomID geom, dReal aabb[6])
    int dCreateGeomClass (const dGeomClass *classptr)
    void * dGeomGetClassData (dGeomID)
    dGeomID dCreateGeom (int classnum)
    void dSetColliderOverride (int i, int j, dColliderFn *fn)

        # ????
    dGeomID dCreateGeomGroup (dSpaceID space)

    dReal *dGeomGetSpaceAABB (dGeomID)

    void dGeomGroupAdd (dGeomID group, dGeomID x)
    void dGeomGroupRemove (dGeomID group, dGeomID x)
    int dGeomGroupGetNumGeoms (dGeomID group)
    dGeomID dGeomGroupGetGeom (dGeomID group, int i)

    dGeomID dCreateGeomTransform (dSpaceID space)
    void dGeomTransformSetGeom (dGeomID g, dGeomID obj)
    dGeomID dGeomTransformGetGeom (dGeomID g)
    void dGeomTransformSetCleanup (dGeomID g, int mode)
    int dGeomTransformGetCleanup (dGeomID g)
    void dGeomTransformSetInfo (dGeomID g, int mode)
    int dGeomTransformGetInfo (dGeomID g)


    # Trimesh
    dTriMeshDataID dGeomTriMeshDataCreate()
    void dGeomTriMeshDataDestroy(dTriMeshDataID g)
    void dGeomTriMeshDataBuildSingle1 (dTriMeshDataID g, void* Vertices,
                                int VertexStride, int VertexCount,
                                void* Indices, int IndexCount,
                                int TriStride, void* Normals)

    void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
                                 dReal* Vertices, int VertexCount,
                                 int* Indices, int IndexCount)

    dGeomID dCreateTriMesh (dSpaceID space, dTriMeshDataID Data,
                            void* Callback,
                            void* ArrayCallback,
                            void* RayCallback)

    void dGeomTriMeshSetData (dGeomID g, dTriMeshDataID Data)

    void dGeomTriMeshClearTCCache (dGeomID g)

    void dGeomTriMeshGetTriangle (dGeomID g, int Index, dVector3 *v0,
                                  dVector3 *v1, dVector3 *v2)

    int dGeomTriMeshGetTriangleCount (dGeomID g)

    void dGeomTriMeshGetPoint (dGeomID g, int Index, dReal u, dReal v,
                               dVector3 Out)

    void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
    int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)
