# Julia wrapper for header: PhysicsClientC_API.h
# Automatically generated using Clang.jl


function b3DisconnectSharedMemory(physClient)
    ccall((:b3DisconnectSharedMemory, libclang), Cvoid, (b3PhysicsClientHandle,), physClient)
end

function b3CanSubmitCommand(physClient)
    ccall((:b3CanSubmitCommand, libclang), Cint, (b3PhysicsClientHandle,), physClient)
end

function b3SubmitClientCommandAndWaitStatus(physClient, commandHandle)
    ccall((:b3SubmitClientCommandAndWaitStatus, libclang), b3SharedMemoryStatusHandle, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle), physClient, commandHandle)
end

function b3SubmitClientCommand(physClient, commandHandle)
    ccall((:b3SubmitClientCommand, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle), physClient, commandHandle)
end

function b3ProcessServerStatus(physClient)
    ccall((:b3ProcessServerStatus, libclang), b3SharedMemoryStatusHandle, (b3PhysicsClientHandle,), physClient)
end

function b3GetStatusType(statusHandle)
    ccall((:b3GetStatusType, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3CreateCustomCommand(physClient)
    ccall((:b3CreateCustomCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3CustomCommandLoadPlugin(commandHandle, pluginPath)
    ccall((:b3CustomCommandLoadPlugin, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cstring), commandHandle, pluginPath)
end

function b3CustomCommandLoadPluginSetPostFix(commandHandle, postFix)
    ccall((:b3CustomCommandLoadPluginSetPostFix, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cstring), commandHandle, postFix)
end

function b3GetStatusPluginUniqueId(statusHandle)
    ccall((:b3GetStatusPluginUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3GetStatusPluginCommandResult(statusHandle)
    ccall((:b3GetStatusPluginCommandResult, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3CustomCommandUnloadPlugin(commandHandle, pluginUniqueId)
    ccall((:b3CustomCommandUnloadPlugin, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, pluginUniqueId)
end

function b3CustomCommandExecutePluginCommand(commandHandle, pluginUniqueId, textArguments)
    ccall((:b3CustomCommandExecutePluginCommand, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cstring), commandHandle, pluginUniqueId, textArguments)
end

function b3CustomCommandExecuteAddIntArgument(commandHandle, intVal)
    ccall((:b3CustomCommandExecuteAddIntArgument, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, intVal)
end

function b3CustomCommandExecuteAddFloatArgument(commandHandle, floatVal)
    ccall((:b3CustomCommandExecuteAddFloatArgument, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat), commandHandle, floatVal)
end

function b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, bodyIndicesCapacity)
    ccall((:b3GetStatusBodyIndices, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cint}, Cint), statusHandle, bodyIndicesOut, bodyIndicesCapacity)
end

function b3GetStatusBodyIndex(statusHandle)
    ccall((:b3GetStatusBodyIndex, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3GetStatusActualState(statusHandle, bodyUniqueId, numDegreeOfFreedomQ, numDegreeOfFreedomU, rootLocalInertialFrame, actualStateQ, actualStateQdot, jointReactionForces)
    ccall((:b3GetStatusActualState, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}), statusHandle, bodyUniqueId, numDegreeOfFreedomQ, numDegreeOfFreedomU, rootLocalInertialFrame, actualStateQ, actualStateQdot, jointReactionForces)
end

function b3GetStatusActualState2(statusHandle, bodyUniqueId, numLinks, numDegreeOfFreedomQ, numDegreeOfFreedomU, rootLocalInertialFrame, actualStateQ, actualStateQdot, jointReactionForces, linkLocalInertialFrames, jointMotorForces, linkStates, linkWorldVelocities)
    ccall((:b3GetStatusActualState2, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}, Ptr{Ptr{Cdouble}}), statusHandle, bodyUniqueId, numLinks, numDegreeOfFreedomQ, numDegreeOfFreedomU, rootLocalInertialFrame, actualStateQ, actualStateQdot, jointReactionForces, linkLocalInertialFrames, jointMotorForces, linkStates, linkWorldVelocities)
end

function b3RequestCollisionInfoCommandInit(physClient, bodyUniqueId)
    ccall((:b3RequestCollisionInfoCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3GetStatusAABB(statusHandle, linkIndex, aabbMin, aabbMax)
    ccall((:b3GetStatusAABB, libclang), Cint, (b3SharedMemoryStatusHandle, Cint, Ptr{Cdouble}, Ptr{Cdouble}), statusHandle, linkIndex, aabbMin, aabbMax)
end

function b3InitSyncBodyInfoCommand(physClient)
    ccall((:b3InitSyncBodyInfoCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitRemoveBodyCommand(physClient, bodyUniqueId)
    ccall((:b3InitRemoveBodyCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3GetNumBodies(physClient)
    ccall((:b3GetNumBodies, libclang), Cint, (b3PhysicsClientHandle,), physClient)
end

function b3GetBodyUniqueId(physClient, serialIndex)
    ccall((:b3GetBodyUniqueId, libclang), Cint, (b3PhysicsClientHandle, Cint), physClient, serialIndex)
end

function b3GetBodyInfo(physClient, bodyUniqueId, info)
    ccall((:b3GetBodyInfo, libclang), Cint, (b3PhysicsClientHandle, Cint, Ptr{b3BodyInfo}), physClient, bodyUniqueId, info)
end

function b3GetNumJoints(physClient, bodyUniqueId)
    ccall((:b3GetNumJoints, libclang), Cint, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3GetNumDofs(physClient, bodyUniqueId)
    ccall((:b3GetNumDofs, libclang), Cint, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3ComputeDofCount(physClient, bodyUniqueId)
    ccall((:b3ComputeDofCount, libclang), Cint, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3GetJointInfo(physClient, bodyUniqueId, jointIndex, info)
    ccall((:b3GetJointInfo, libclang), Cint, (b3PhysicsClientHandle, Cint, Cint, Ptr{b3JointInfo}), physClient, bodyUniqueId, jointIndex, info)
end

function b3InitSyncUserDataCommand(physClient)
    ccall((:b3InitSyncUserDataCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitAddUserDataCommand(physClient, bodyUniqueId, linkIndex, visualShapeIndex, key, valueType, valueLength, valueData)
    ccall((:b3InitAddUserDataCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint, Cint, Cstring, UserDataValueType, Cint, Ptr{Cvoid}), physClient, bodyUniqueId, linkIndex, visualShapeIndex, key, valueType, valueLength, valueData)
end

function b3InitRemoveUserDataCommand(physClient, userDataId)
    ccall((:b3InitRemoveUserDataCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, userDataId)
end

function b3GetUserData(physClient, userDataId, valueOut)
    ccall((:b3GetUserData, libclang), Cint, (b3PhysicsClientHandle, Cint, Ptr{b3UserDataValue}), physClient, userDataId, valueOut)
end

function b3GetUserDataId(physClient, bodyUniqueId, linkIndex, visualShapeIndex, key)
    ccall((:b3GetUserDataId, libclang), Cint, (b3PhysicsClientHandle, Cint, Cint, Cint, Cstring), physClient, bodyUniqueId, linkIndex, visualShapeIndex, key)
end

function b3GetUserDataIdFromStatus(statusHandle)
    ccall((:b3GetUserDataIdFromStatus, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3GetNumUserData(physClient, bodyUniqueId)
    ccall((:b3GetNumUserData, libclang), Cint, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3GetUserDataInfo(physClient, bodyUniqueId, userDataIndex, keyOut, userDataIdOut, linkIndexOut, visualShapeIndexOut)
    ccall((:b3GetUserDataInfo, libclang), Cvoid, (b3PhysicsClientHandle, Cint, Cint, Ptr{Cstring}, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), physClient, bodyUniqueId, userDataIndex, keyOut, userDataIdOut, linkIndexOut, visualShapeIndexOut)
end

function b3GetDynamicsInfoCommandInit(physClient, bodyUniqueId, linkIndex)
    ccall((:b3GetDynamicsInfoCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint), physClient, bodyUniqueId, linkIndex)
end

function b3GetDynamicsInfoCommandInit2(commandHandle, bodyUniqueId, linkIndex)
    ccall((:b3GetDynamicsInfoCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, bodyUniqueId, linkIndex)
end

function b3GetDynamicsInfo(statusHandle, info)
    ccall((:b3GetDynamicsInfo, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{b3DynamicsInfo}), statusHandle, info)
end

function b3InitChangeDynamicsInfo(physClient)
    ccall((:b3InitChangeDynamicsInfo, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitChangeDynamicsInfo2(commandHandle)
    ccall((:b3InitChangeDynamicsInfo2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3ChangeDynamicsInfoSetMass(commandHandle, bodyUniqueId, linkIndex, mass)
    ccall((:b3ChangeDynamicsInfoSetMass, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, mass)
end

function b3ChangeDynamicsInfoSetLocalInertiaDiagonal(commandHandle, bodyUniqueId, linkIndex, localInertiaDiagonal)
    ccall((:b3ChangeDynamicsInfoSetLocalInertiaDiagonal, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}), commandHandle, bodyUniqueId, linkIndex, localInertiaDiagonal)
end

function b3ChangeDynamicsInfoSetAnisotropicFriction(commandHandle, bodyUniqueId, linkIndex, anisotropicFriction)
    ccall((:b3ChangeDynamicsInfoSetAnisotropicFriction, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}), commandHandle, bodyUniqueId, linkIndex, anisotropicFriction)
end

function b3ChangeDynamicsInfoSetLateralFriction(commandHandle, bodyUniqueId, linkIndex, lateralFriction)
    ccall((:b3ChangeDynamicsInfoSetLateralFriction, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, lateralFriction)
end

function b3ChangeDynamicsInfoSetSpinningFriction(commandHandle, bodyUniqueId, linkIndex, friction)
    ccall((:b3ChangeDynamicsInfoSetSpinningFriction, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, friction)
end

function b3ChangeDynamicsInfoSetRollingFriction(commandHandle, bodyUniqueId, linkIndex, friction)
    ccall((:b3ChangeDynamicsInfoSetRollingFriction, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, friction)
end

function b3ChangeDynamicsInfoSetRestitution(commandHandle, bodyUniqueId, linkIndex, restitution)
    ccall((:b3ChangeDynamicsInfoSetRestitution, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, restitution)
end

function b3ChangeDynamicsInfoSetLinearDamping(commandHandle, bodyUniqueId, linearDamping)
    ccall((:b3ChangeDynamicsInfoSetLinearDamping, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, bodyUniqueId, linearDamping)
end

function b3ChangeDynamicsInfoSetAngularDamping(commandHandle, bodyUniqueId, angularDamping)
    ccall((:b3ChangeDynamicsInfoSetAngularDamping, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, bodyUniqueId, angularDamping)
end

function b3ChangeDynamicsInfoSetJointDamping(commandHandle, bodyUniqueId, linkIndex, jointDamping)
    ccall((:b3ChangeDynamicsInfoSetJointDamping, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, jointDamping)
end

function b3ChangeDynamicsInfoSetContactStiffnessAndDamping(commandHandle, bodyUniqueId, linkIndex, contactStiffness, contactDamping)
    ccall((:b3ChangeDynamicsInfoSetContactStiffnessAndDamping, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble, Cdouble), commandHandle, bodyUniqueId, linkIndex, contactStiffness, contactDamping)
end

function b3ChangeDynamicsInfoSetFrictionAnchor(commandHandle, bodyUniqueId, linkIndex, frictionAnchor)
    ccall((:b3ChangeDynamicsInfoSetFrictionAnchor, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cint), commandHandle, bodyUniqueId, linkIndex, frictionAnchor)
end

function b3ChangeDynamicsInfoSetCcdSweptSphereRadius(commandHandle, bodyUniqueId, linkIndex, ccdSweptSphereRadius)
    ccall((:b3ChangeDynamicsInfoSetCcdSweptSphereRadius, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, ccdSweptSphereRadius)
end

function b3ChangeDynamicsInfoSetContactProcessingThreshold(commandHandle, bodyUniqueId, linkIndex, contactProcessingThreshold)
    ccall((:b3ChangeDynamicsInfoSetContactProcessingThreshold, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint, Cdouble), commandHandle, bodyUniqueId, linkIndex, contactProcessingThreshold)
end

function b3ChangeDynamicsInfoSetActivationState(commandHandle, bodyUniqueId, activationState)
    ccall((:b3ChangeDynamicsInfoSetActivationState, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, bodyUniqueId, activationState)
end

function b3ChangeDynamicsInfoSetMaxJointVelocity(commandHandle, bodyUniqueId, maxJointVelocity)
    ccall((:b3ChangeDynamicsInfoSetMaxJointVelocity, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, bodyUniqueId, maxJointVelocity)
end

function b3InitCreateUserConstraintCommand(physClient, parentBodyUniqueId, parentJointIndex, childBodyUniqueId, childJointIndex, info)
    ccall((:b3InitCreateUserConstraintCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint, Cint, Cint, Ptr{b3JointInfo}), physClient, parentBodyUniqueId, parentJointIndex, childBodyUniqueId, childJointIndex, info)
end

function b3InitCreateUserConstraintCommand2(commandHandle, parentBodyUniqueId, parentJointIndex, childBodyUniqueId, childJointIndex, info)
    ccall((:b3InitCreateUserConstraintCommand2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cint, Cint, Cint, Cint, Ptr{b3JointInfo}), commandHandle, parentBodyUniqueId, parentJointIndex, childBodyUniqueId, childJointIndex, info)
end

function b3GetStatusUserConstraintUniqueId(statusHandle)
    ccall((:b3GetStatusUserConstraintUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3InitChangeUserConstraintCommand(physClient, userConstraintUniqueId)
    ccall((:b3InitChangeUserConstraintCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, userConstraintUniqueId)
end

function b3InitChangeUserConstraintSetPivotInB(commandHandle, jointChildPivot)
    ccall((:b3InitChangeUserConstraintSetPivotInB, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, jointChildPivot)
end

function b3InitChangeUserConstraintSetFrameInB(commandHandle, jointChildFrameOrn)
    ccall((:b3InitChangeUserConstraintSetFrameInB, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, jointChildFrameOrn)
end

function b3InitChangeUserConstraintSetMaxForce(commandHandle, maxAppliedForce)
    ccall((:b3InitChangeUserConstraintSetMaxForce, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, maxAppliedForce)
end

function b3InitChangeUserConstraintSetGearRatio(commandHandle, gearRatio)
    ccall((:b3InitChangeUserConstraintSetGearRatio, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, gearRatio)
end

function b3InitChangeUserConstraintSetGearAuxLink(commandHandle, gearAuxLink)
    ccall((:b3InitChangeUserConstraintSetGearAuxLink, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, gearAuxLink)
end

function b3InitChangeUserConstraintSetRelativePositionTarget(commandHandle, relativePositionTarget)
    ccall((:b3InitChangeUserConstraintSetRelativePositionTarget, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, relativePositionTarget)
end

function b3InitChangeUserConstraintSetERP(commandHandle, erp)
    ccall((:b3InitChangeUserConstraintSetERP, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, erp)
end

function b3InitRemoveUserConstraintCommand(physClient, userConstraintUniqueId)
    ccall((:b3InitRemoveUserConstraintCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, userConstraintUniqueId)
end

function b3GetNumUserConstraints(physClient)
    ccall((:b3GetNumUserConstraints, libclang), Cint, (b3PhysicsClientHandle,), physClient)
end

function b3InitGetUserConstraintStateCommand(physClient, constraintUniqueId)
    ccall((:b3InitGetUserConstraintStateCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, constraintUniqueId)
end

function b3GetStatusUserConstraintState(statusHandle, constraintState)
    ccall((:b3GetStatusUserConstraintState, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{b3UserConstraintState}), statusHandle, constraintState)
end

function b3GetUserConstraintInfo(physClient, constraintUniqueId, info)
    ccall((:b3GetUserConstraintInfo, libclang), Cint, (b3PhysicsClientHandle, Cint, Ptr{b3UserConstraint}), physClient, constraintUniqueId, info)
end

function b3GetUserConstraintId(physClient, serialIndex)
    ccall((:b3GetUserConstraintId, libclang), Cint, (b3PhysicsClientHandle, Cint), physClient, serialIndex)
end

function b3InitRequestDebugLinesCommand(physClient, debugMode)
    ccall((:b3InitRequestDebugLinesCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, debugMode)
end

function b3GetDebugLines(physClient, lines)
    ccall((:b3GetDebugLines, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3DebugLines}), physClient, lines)
end

function b3InitConfigureOpenGLVisualizer(physClient)
    ccall((:b3InitConfigureOpenGLVisualizer, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitConfigureOpenGLVisualizer2(commandHandle)
    ccall((:b3InitConfigureOpenGLVisualizer2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3ConfigureOpenGLVisualizerSetVisualizationFlags(commandHandle, flag, enabled)
    ccall((:b3ConfigureOpenGLVisualizerSetVisualizationFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, flag, enabled)
end

function b3ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, cameraDistance, cameraPitch, cameraYaw, cameraTargetPosition)
    ccall((:b3ConfigureOpenGLVisualizerSetViewMatrix, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat, Cfloat, Cfloat, Ptr{Cfloat}), commandHandle, cameraDistance, cameraPitch, cameraYaw, cameraTargetPosition)
end

function b3InitRequestOpenGLVisualizerCameraCommand(physClient)
    ccall((:b3InitRequestOpenGLVisualizerCameraCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3GetStatusOpenGLVisualizerCamera(statusHandle, camera)
    ccall((:b3GetStatusOpenGLVisualizerCamera, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{b3OpenGLVisualizerCameraInfo}), statusHandle, camera)
end

function b3InitUserDebugDrawAddLine3D(physClient, fromXYZ, toXYZ, colorRGB, lineWidth, lifeTime)
    ccall((:b3InitUserDebugDrawAddLine3D, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Cdouble), physClient, fromXYZ, toXYZ, colorRGB, lineWidth, lifeTime)
end

function b3InitUserDebugDrawAddText3D(physClient, txt, positionXYZ, colorRGB, textSize, lifeTime)
    ccall((:b3InitUserDebugDrawAddText3D, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring, Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Cdouble), physClient, txt, positionXYZ, colorRGB, textSize, lifeTime)
end

function b3UserDebugTextSetOptionFlags(commandHandle, optionFlags)
    ccall((:b3UserDebugTextSetOptionFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, optionFlags)
end

function b3UserDebugTextSetOrientation(commandHandle, orientation)
    ccall((:b3UserDebugTextSetOrientation, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, orientation)
end

function b3UserDebugItemSetReplaceItemUniqueId(commandHandle, replaceItem)
    ccall((:b3UserDebugItemSetReplaceItemUniqueId, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, replaceItem)
end

function b3UserDebugItemSetParentObject(commandHandle, objectUniqueId, linkIndex)
    ccall((:b3UserDebugItemSetParentObject, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, objectUniqueId, linkIndex)
end

function b3InitUserDebugAddParameter(physClient, txt, rangeMin, rangeMax, startValue)
    ccall((:b3InitUserDebugAddParameter, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring, Cdouble, Cdouble, Cdouble), physClient, txt, rangeMin, rangeMax, startValue)
end

function b3InitUserDebugReadParameter(physClient, debugItemUniqueId)
    ccall((:b3InitUserDebugReadParameter, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, debugItemUniqueId)
end

function b3GetStatusDebugParameterValue(statusHandle, paramValue)
    ccall((:b3GetStatusDebugParameterValue, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cdouble}), statusHandle, paramValue)
end

function b3InitUserDebugDrawRemove(physClient, debugItemUniqueId)
    ccall((:b3InitUserDebugDrawRemove, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, debugItemUniqueId)
end

function b3InitUserDebugDrawRemoveAll(physClient)
    ccall((:b3InitUserDebugDrawRemoveAll, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitDebugDrawingCommand(physClient)
    ccall((:b3InitDebugDrawingCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3SetDebugObjectColor(commandHandle, objectUniqueId, linkIndex, objectColorRGB)
    ccall((:b3SetDebugObjectColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}), commandHandle, objectUniqueId, linkIndex, objectColorRGB)
end

function b3RemoveDebugObjectColor(commandHandle, objectUniqueId, linkIndex)
    ccall((:b3RemoveDebugObjectColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, objectUniqueId, linkIndex)
end

function b3GetDebugItemUniqueId(statusHandle)
    ccall((:b3GetDebugItemUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3InitRequestCameraImage(physClient)
    ccall((:b3InitRequestCameraImage, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitRequestCameraImage2(commandHandle)
    ccall((:b3InitRequestCameraImage2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3RequestCameraImageSetCameraMatrices(commandHandle, viewMatrix, projectionMatrix)
    ccall((:b3RequestCameraImageSetCameraMatrices, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cfloat}, Ptr{Cfloat}), commandHandle, viewMatrix, projectionMatrix)
end

function b3RequestCameraImageSetPixelResolution(commandHandle, width, height)
    ccall((:b3RequestCameraImageSetPixelResolution, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, width, height)
end

function b3RequestCameraImageSetLightDirection(commandHandle, lightDirection)
    ccall((:b3RequestCameraImageSetLightDirection, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cfloat}), commandHandle, lightDirection)
end

function b3RequestCameraImageSetLightColor(commandHandle, lightColor)
    ccall((:b3RequestCameraImageSetLightColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cfloat}), commandHandle, lightColor)
end

function b3RequestCameraImageSetLightDistance(commandHandle, lightDistance)
    ccall((:b3RequestCameraImageSetLightDistance, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat), commandHandle, lightDistance)
end

function b3RequestCameraImageSetLightAmbientCoeff(commandHandle, lightAmbientCoeff)
    ccall((:b3RequestCameraImageSetLightAmbientCoeff, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat), commandHandle, lightAmbientCoeff)
end

function b3RequestCameraImageSetLightDiffuseCoeff(commandHandle, lightDiffuseCoeff)
    ccall((:b3RequestCameraImageSetLightDiffuseCoeff, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat), commandHandle, lightDiffuseCoeff)
end

function b3RequestCameraImageSetLightSpecularCoeff(commandHandle, lightSpecularCoeff)
    ccall((:b3RequestCameraImageSetLightSpecularCoeff, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat), commandHandle, lightSpecularCoeff)
end

function b3RequestCameraImageSetShadow(commandHandle, hasShadow)
    ccall((:b3RequestCameraImageSetShadow, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, hasShadow)
end

function b3RequestCameraImageSelectRenderer(commandHandle, renderer)
    ccall((:b3RequestCameraImageSelectRenderer, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, renderer)
end

function b3RequestCameraImageSetFlags(commandHandle, flags)
    ccall((:b3RequestCameraImageSetFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3GetCameraImageData(physClient, imageData)
    ccall((:b3GetCameraImageData, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3CameraImageData}), physClient, imageData)
end

function b3RequestCameraImageSetProjectiveTextureMatrices(commandHandle, viewMatrix, projectionMatrix)
    ccall((:b3RequestCameraImageSetProjectiveTextureMatrices, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cfloat}, Ptr{Cfloat}), commandHandle, viewMatrix, projectionMatrix)
end

function b3ComputeViewMatrixFromPositions(cameraPosition, cameraTargetPosition, cameraUp, viewMatrix)
    ccall((:b3ComputeViewMatrixFromPositions, libclang), Cvoid, (Ptr{Cfloat}, Ptr{Cfloat}, Ptr{Cfloat}, Ptr{Cfloat}), cameraPosition, cameraTargetPosition, cameraUp, viewMatrix)
end

function b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxis, viewMatrix)
    ccall((:b3ComputeViewMatrixFromYawPitchRoll, libclang), Cvoid, (Ptr{Cfloat}, Cfloat, Cfloat, Cfloat, Cfloat, Cint, Ptr{Cfloat}), cameraTargetPosition, distance, yaw, pitch, roll, upAxis, viewMatrix)
end

function b3ComputePositionFromViewMatrix(viewMatrix, cameraPosition, cameraTargetPosition, cameraUp)
    ccall((:b3ComputePositionFromViewMatrix, libclang), Cvoid, (Ptr{Cfloat}, Ptr{Cfloat}, Ptr{Cfloat}, Ptr{Cfloat}), viewMatrix, cameraPosition, cameraTargetPosition, cameraUp)
end

function b3ComputeProjectionMatrix(left, right, bottom, top, nearVal, farVal, projectionMatrix)
    ccall((:b3ComputeProjectionMatrix, libclang), Cvoid, (Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Ptr{Cfloat}), left, right, bottom, top, nearVal, farVal, projectionMatrix)
end

function b3ComputeProjectionMatrixFOV(fov, aspect, nearVal, farVal, projectionMatrix)
    ccall((:b3ComputeProjectionMatrixFOV, libclang), Cvoid, (Cfloat, Cfloat, Cfloat, Cfloat, Ptr{Cfloat}), fov, aspect, nearVal, farVal, projectionMatrix)
end

function b3RequestCameraImageSetViewMatrix(commandHandle, cameraPosition, cameraTargetPosition, cameraUp)
    ccall((:b3RequestCameraImageSetViewMatrix, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cfloat}, Ptr{Cfloat}, Ptr{Cfloat}), commandHandle, cameraPosition, cameraTargetPosition, cameraUp)
end

function b3RequestCameraImageSetViewMatrix2(commandHandle, cameraTargetPosition, distance, yaw, pitch, roll, upAxis)
    ccall((:b3RequestCameraImageSetViewMatrix2, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cfloat}, Cfloat, Cfloat, Cfloat, Cfloat, Cint), commandHandle, cameraTargetPosition, distance, yaw, pitch, roll, upAxis)
end

function b3RequestCameraImageSetProjectionMatrix(commandHandle, left, right, bottom, top, nearVal, farVal)
    ccall((:b3RequestCameraImageSetProjectionMatrix, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat), commandHandle, left, right, bottom, top, nearVal, farVal)
end

function b3RequestCameraImageSetFOVProjectionMatrix(commandHandle, fov, aspect, nearVal, farVal)
    ccall((:b3RequestCameraImageSetFOVProjectionMatrix, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cfloat, Cfloat, Cfloat, Cfloat), commandHandle, fov, aspect, nearVal, farVal)
end

function b3InitRequestContactPointInformation(physClient)
    ccall((:b3InitRequestContactPointInformation, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3SetContactFilterBodyA(commandHandle, bodyUniqueIdA)
    ccall((:b3SetContactFilterBodyA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyUniqueIdA)
end

function b3SetContactFilterBodyB(commandHandle, bodyUniqueIdB)
    ccall((:b3SetContactFilterBodyB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyUniqueIdB)
end

function b3SetContactFilterLinkA(commandHandle, linkIndexA)
    ccall((:b3SetContactFilterLinkA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, linkIndexA)
end

function b3SetContactFilterLinkB(commandHandle, linkIndexB)
    ccall((:b3SetContactFilterLinkB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, linkIndexB)
end

function b3GetContactPointInformation(physClient, contactPointData)
    ccall((:b3GetContactPointInformation, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3ContactInformation}), physClient, contactPointData)
end

function b3InitClosestDistanceQuery(physClient)
    ccall((:b3InitClosestDistanceQuery, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3SetClosestDistanceFilterBodyA(commandHandle, bodyUniqueIdA)
    ccall((:b3SetClosestDistanceFilterBodyA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyUniqueIdA)
end

function b3SetClosestDistanceFilterLinkA(commandHandle, linkIndexA)
    ccall((:b3SetClosestDistanceFilterLinkA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, linkIndexA)
end

function b3SetClosestDistanceFilterBodyB(commandHandle, bodyUniqueIdB)
    ccall((:b3SetClosestDistanceFilterBodyB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyUniqueIdB)
end

function b3SetClosestDistanceFilterLinkB(commandHandle, linkIndexB)
    ccall((:b3SetClosestDistanceFilterLinkB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, linkIndexB)
end

function b3SetClosestDistanceThreshold(commandHandle, distance)
    ccall((:b3SetClosestDistanceThreshold, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, distance)
end

function b3SetClosestDistanceFilterCollisionShapeA(commandHandle, collisionShapeA)
    ccall((:b3SetClosestDistanceFilterCollisionShapeA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, collisionShapeA)
end

function b3SetClosestDistanceFilterCollisionShapeB(commandHandle, collisionShapeB)
    ccall((:b3SetClosestDistanceFilterCollisionShapeB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, collisionShapeB)
end

function b3SetClosestDistanceFilterCollisionShapePositionA(commandHandle, collisionShapePositionA)
    ccall((:b3SetClosestDistanceFilterCollisionShapePositionA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, collisionShapePositionA)
end

function b3SetClosestDistanceFilterCollisionShapePositionB(commandHandle, collisionShapePositionB)
    ccall((:b3SetClosestDistanceFilterCollisionShapePositionB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, collisionShapePositionB)
end

function b3SetClosestDistanceFilterCollisionShapeOrientationA(commandHandle, collisionShapeOrientationA)
    ccall((:b3SetClosestDistanceFilterCollisionShapeOrientationA, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, collisionShapeOrientationA)
end

function b3SetClosestDistanceFilterCollisionShapeOrientationB(commandHandle, collisionShapeOrientationB)
    ccall((:b3SetClosestDistanceFilterCollisionShapeOrientationB, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, collisionShapeOrientationB)
end

function b3GetClosestPointInformation(physClient, contactPointInfo)
    ccall((:b3GetClosestPointInformation, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3ContactInformation}), physClient, contactPointInfo)
end

function b3InitAABBOverlapQuery(physClient, aabbMin, aabbMax)
    ccall((:b3InitAABBOverlapQuery, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Ptr{Cdouble}, Ptr{Cdouble}), physClient, aabbMin, aabbMax)
end

function b3GetAABBOverlapResults(physClient, data)
    ccall((:b3GetAABBOverlapResults, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3AABBOverlapData}), physClient, data)
end

function b3InitRequestVisualShapeInformation(physClient, bodyUniqueIdA)
    ccall((:b3InitRequestVisualShapeInformation, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueIdA)
end

function b3GetVisualShapeInformation(physClient, visualShapeInfo)
    ccall((:b3GetVisualShapeInformation, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3VisualShapeInformation}), physClient, visualShapeInfo)
end

function b3InitRequestCollisionShapeInformation(physClient, bodyUniqueId, linkIndex)
    ccall((:b3InitRequestCollisionShapeInformation, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint), physClient, bodyUniqueId, linkIndex)
end

function b3GetCollisionShapeInformation(physClient, collisionShapeInfo)
    ccall((:b3GetCollisionShapeInformation, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3CollisionShapeInformation}), physClient, collisionShapeInfo)
end

function b3InitLoadTexture(physClient, filename)
    ccall((:b3InitLoadTexture, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, filename)
end

function b3GetStatusTextureUniqueId(statusHandle)
    ccall((:b3GetStatusTextureUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3CreateChangeTextureCommandInit(physClient, textureUniqueId, width, height, rgbPixels)
    ccall((:b3CreateChangeTextureCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint, Cint, Cstring), physClient, textureUniqueId, width, height, rgbPixels)
end

function b3InitUpdateVisualShape(physClient, bodyUniqueId, jointIndex, shapeIndex, textureUniqueId)
    ccall((:b3InitUpdateVisualShape, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint, Cint, Cint), physClient, bodyUniqueId, jointIndex, shapeIndex, textureUniqueId)
end

function b3InitUpdateVisualShape2(physClient, bodyUniqueId, jointIndex, shapeIndex)
    ccall((:b3InitUpdateVisualShape2, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint, Cint), physClient, bodyUniqueId, jointIndex, shapeIndex)
end

function b3UpdateVisualShapeTexture(commandHandle, textureUniqueId)
    ccall((:b3UpdateVisualShapeTexture, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, textureUniqueId)
end

function b3UpdateVisualShapeRGBAColor(commandHandle, rgbaColor)
    ccall((:b3UpdateVisualShapeRGBAColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, rgbaColor)
end

function b3UpdateVisualShapeSpecularColor(commandHandle, specularColor)
    ccall((:b3UpdateVisualShapeSpecularColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, specularColor)
end

function b3InitPhysicsParamCommand(physClient)
    ccall((:b3InitPhysicsParamCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitPhysicsParamCommand2(commandHandle)
    ccall((:b3InitPhysicsParamCommand2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3PhysicsParamSetGravity(commandHandle, gravx, gravy, gravz)
    ccall((:b3PhysicsParamSetGravity, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble), commandHandle, gravx, gravy, gravz)
end

function b3PhysicsParamSetTimeStep(commandHandle, timeStep)
    ccall((:b3PhysicsParamSetTimeStep, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, timeStep)
end

function b3PhysicsParamSetDefaultContactERP(commandHandle, defaultContactERP)
    ccall((:b3PhysicsParamSetDefaultContactERP, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, defaultContactERP)
end

function b3PhysicsParamSetDefaultNonContactERP(commandHandle, defaultNonContactERP)
    ccall((:b3PhysicsParamSetDefaultNonContactERP, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, defaultNonContactERP)
end

function b3PhysicsParamSetDefaultFrictionERP(commandHandle, frictionERP)
    ccall((:b3PhysicsParamSetDefaultFrictionERP, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, frictionERP)
end

function b3PhysicsParamSetDefaultGlobalCFM(commandHandle, defaultGlobalCFM)
    ccall((:b3PhysicsParamSetDefaultGlobalCFM, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, defaultGlobalCFM)
end

function b3PhysicsParamSetDefaultFrictionCFM(commandHandle, frictionCFM)
    ccall((:b3PhysicsParamSetDefaultFrictionCFM, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, frictionCFM)
end

function b3PhysicsParamSetNumSubSteps(commandHandle, numSubSteps)
    ccall((:b3PhysicsParamSetNumSubSteps, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, numSubSteps)
end

function b3PhysicsParamSetRealTimeSimulation(commandHandle, enableRealTimeSimulation)
    ccall((:b3PhysicsParamSetRealTimeSimulation, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, enableRealTimeSimulation)
end

function b3PhysicsParamSetNumSolverIterations(commandHandle, numSolverIterations)
    ccall((:b3PhysicsParamSetNumSolverIterations, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, numSolverIterations)
end

function b3PhysicsParamSetCollisionFilterMode(commandHandle, filterMode)
    ccall((:b3PhysicsParamSetCollisionFilterMode, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, filterMode)
end

function b3PhysicsParamSetUseSplitImpulse(commandHandle, useSplitImpulse)
    ccall((:b3PhysicsParamSetUseSplitImpulse, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, useSplitImpulse)
end

function b3PhysicsParamSetSplitImpulsePenetrationThreshold(commandHandle, splitImpulsePenetrationThreshold)
    ccall((:b3PhysicsParamSetSplitImpulsePenetrationThreshold, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, splitImpulsePenetrationThreshold)
end

function b3PhysicsParamSetContactBreakingThreshold(commandHandle, contactBreakingThreshold)
    ccall((:b3PhysicsParamSetContactBreakingThreshold, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, contactBreakingThreshold)
end

function b3PhysicsParamSetMaxNumCommandsPer1ms(commandHandle, maxNumCmdPer1ms)
    ccall((:b3PhysicsParamSetMaxNumCommandsPer1ms, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, maxNumCmdPer1ms)
end

function b3PhysicsParamSetEnableFileCaching(commandHandle, enableFileCaching)
    ccall((:b3PhysicsParamSetEnableFileCaching, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, enableFileCaching)
end

function b3PhysicsParamSetRestitutionVelocityThreshold(commandHandle, restitutionVelocityThreshold)
    ccall((:b3PhysicsParamSetRestitutionVelocityThreshold, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, restitutionVelocityThreshold)
end

function b3PhysicsParamSetEnableConeFriction(commandHandle, enableConeFriction)
    ccall((:b3PhysicsParamSetEnableConeFriction, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, enableConeFriction)
end

function b3PhysicsParameterSetDeterministicOverlappingPairs(commandHandle, deterministicOverlappingPairs)
    ccall((:b3PhysicsParameterSetDeterministicOverlappingPairs, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, deterministicOverlappingPairs)
end

function b3PhysicsParameterSetAllowedCcdPenetration(commandHandle, allowedCcdPenetration)
    ccall((:b3PhysicsParameterSetAllowedCcdPenetration, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, allowedCcdPenetration)
end

function b3PhysicsParameterSetJointFeedbackMode(commandHandle, jointFeedbackMode)
    ccall((:b3PhysicsParameterSetJointFeedbackMode, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, jointFeedbackMode)
end

function b3PhysicsParamSetSolverResidualThreshold(commandHandle, solverResidualThreshold)
    ccall((:b3PhysicsParamSetSolverResidualThreshold, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, solverResidualThreshold)
end

function b3PhysicsParamSetContactSlop(commandHandle, contactSlop)
    ccall((:b3PhysicsParamSetContactSlop, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, contactSlop)
end

function b3PhysicsParameterSetEnableSAT(commandHandle, enableSAT)
    ccall((:b3PhysicsParameterSetEnableSAT, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, enableSAT)
end

function b3PhysicsParameterSetConstraintSolverType(commandHandle, constraintSolverType)
    ccall((:b3PhysicsParameterSetConstraintSolverType, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, constraintSolverType)
end

function b3PhysicsParameterSetMinimumSolverIslandSize(commandHandle, minimumSolverIslandSize)
    ccall((:b3PhysicsParameterSetMinimumSolverIslandSize, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, minimumSolverIslandSize)
end

function b3PhysicsParamSetSolverAnalytics(commandHandle, reportSolverAnalytics)
    ccall((:b3PhysicsParamSetSolverAnalytics, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, reportSolverAnalytics)
end

function b3InitRequestPhysicsParamCommand(physClient)
    ccall((:b3InitRequestPhysicsParamCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3GetStatusPhysicsSimulationParameters(statusHandle, params)
    ccall((:b3GetStatusPhysicsSimulationParameters, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{b3PhysicsSimulationParameters}), statusHandle, params)
end

function b3PhysicsParamSetInternalSimFlags(commandHandle, flags)
    ccall((:b3PhysicsParamSetInternalSimFlags, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3InitStepSimulationCommand(physClient)
    ccall((:b3InitStepSimulationCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitStepSimulationCommand2(commandHandle)
    ccall((:b3InitStepSimulationCommand2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3GetStatusForwardDynamicsAnalyticsData(statusHandle, analyticsData)
    ccall((:b3GetStatusForwardDynamicsAnalyticsData, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{b3ForwardDynamicsAnalyticsArgs}), statusHandle, analyticsData)
end

function b3InitResetSimulationCommand(physClient)
    ccall((:b3InitResetSimulationCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitResetSimulationCommand2(commandHandle)
    ccall((:b3InitResetSimulationCommand2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3LoadUrdfCommandInit(physClient, urdfFileName)
    ccall((:b3LoadUrdfCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, urdfFileName)
end

function b3LoadUrdfCommandInit2(commandHandle, urdfFileName)
    ccall((:b3LoadUrdfCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cstring), commandHandle, urdfFileName)
end

function b3LoadUrdfCommandSetStartPosition(commandHandle, startPosX, startPosY, startPosZ)
    ccall((:b3LoadUrdfCommandSetStartPosition, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble), commandHandle, startPosX, startPosY, startPosZ)
end

function b3LoadUrdfCommandSetStartOrientation(commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
    ccall((:b3LoadUrdfCommandSetStartOrientation, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble, Cdouble), commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
end

function b3LoadUrdfCommandSetUseMultiBody(commandHandle, useMultiBody)
    ccall((:b3LoadUrdfCommandSetUseMultiBody, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, useMultiBody)
end

function b3LoadUrdfCommandSetUseFixedBase(commandHandle, useFixedBase)
    ccall((:b3LoadUrdfCommandSetUseFixedBase, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, useFixedBase)
end

function b3LoadUrdfCommandSetFlags(commandHandle, flags)
    ccall((:b3LoadUrdfCommandSetFlags, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3LoadUrdfCommandSetGlobalScaling(commandHandle, globalScaling)
    ccall((:b3LoadUrdfCommandSetGlobalScaling, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, globalScaling)
end

function b3SaveStateCommandInit(physClient)
    ccall((:b3SaveStateCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3InitRemoveStateCommand(physClient, stateId)
    ccall((:b3InitRemoveStateCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, stateId)
end

function b3GetStatusGetStateId(statusHandle)
    ccall((:b3GetStatusGetStateId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3LoadStateCommandInit(physClient)
    ccall((:b3LoadStateCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3LoadStateSetStateId(commandHandle, stateId)
    ccall((:b3LoadStateSetStateId, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, stateId)
end

function b3LoadStateSetFileName(commandHandle, fileName)
    ccall((:b3LoadStateSetFileName, libclang), Cint, (b3SharedMemoryCommandHandle, Cstring), commandHandle, fileName)
end

function b3LoadBulletCommandInit(physClient, fileName)
    ccall((:b3LoadBulletCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, fileName)
end

function b3SaveBulletCommandInit(physClient, fileName)
    ccall((:b3SaveBulletCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, fileName)
end

function b3LoadMJCFCommandInit(physClient, fileName)
    ccall((:b3LoadMJCFCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, fileName)
end

function b3LoadMJCFCommandInit2(commandHandle, fileName)
    ccall((:b3LoadMJCFCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cstring), commandHandle, fileName)
end

function b3LoadMJCFCommandSetFlags(commandHandle, flags)
    ccall((:b3LoadMJCFCommandSetFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3CalculateInverseDynamicsCommandInit(physClient, bodyUniqueId, jointPositionsQ, jointVelocitiesQdot, jointAccelerations)
    ccall((:b3CalculateInverseDynamicsCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), physClient, bodyUniqueId, jointPositionsQ, jointVelocitiesQdot, jointAccelerations)
end

function b3CalculateInverseDynamicsCommandInit2(physClient, bodyUniqueId, jointPositionsQ, dofCountQ, jointVelocitiesQdot, jointAccelerations, dofCountQdot)
    ccall((:b3CalculateInverseDynamicsCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Ptr{Cdouble}, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Cint), physClient, bodyUniqueId, jointPositionsQ, dofCountQ, jointVelocitiesQdot, jointAccelerations, dofCountQdot)
end

function b3CalculateInverseDynamicsSetFlags(commandHandle, flags)
    ccall((:b3CalculateInverseDynamicsSetFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3GetStatusInverseDynamicsJointForces(statusHandle, bodyUniqueId, dofCount, jointForces)
    ccall((:b3GetStatusInverseDynamicsJointForces, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}), statusHandle, bodyUniqueId, dofCount, jointForces)
end

function b3CalculateJacobianCommandInit(physClient, bodyUniqueId, linkIndex, localPosition, jointPositionsQ, jointVelocitiesQdot, jointAccelerations)
    ccall((:b3CalculateJacobianCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), physClient, bodyUniqueId, linkIndex, localPosition, jointPositionsQ, jointVelocitiesQdot, jointAccelerations)
end

function b3GetStatusJacobian(statusHandle, dofCount, linearJacobian, angularJacobian)
    ccall((:b3GetStatusJacobian, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cint}, Ptr{Cdouble}, Ptr{Cdouble}), statusHandle, dofCount, linearJacobian, angularJacobian)
end

function b3CalculateMassMatrixCommandInit(physClient, bodyUniqueId, jointPositionsQ, dofCountQ)
    ccall((:b3CalculateMassMatrixCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Ptr{Cdouble}, Cint), physClient, bodyUniqueId, jointPositionsQ, dofCountQ)
end

function b3CalculateMassMatrixSetFlags(commandHandle, flags)
    ccall((:b3CalculateMassMatrixSetFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3GetStatusMassMatrix(physClient, statusHandle, dofCount, massMatrix)
    ccall((:b3GetStatusMassMatrix, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryStatusHandle, Ptr{Cint}, Ptr{Cdouble}), physClient, statusHandle, dofCount, massMatrix)
end

function b3CalculateInverseKinematicsCommandInit(physClient, bodyUniqueId)
    ccall((:b3CalculateInverseKinematicsCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3CalculateInverseKinematicsAddTargetPurePosition(commandHandle, endEffectorLinkIndex, targetPosition)
    ccall((:b3CalculateInverseKinematicsAddTargetPurePosition, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), commandHandle, endEffectorLinkIndex, targetPosition)
end

function b3CalculateInverseKinematicsAddTargetPositionWithOrientation(commandHandle, endEffectorLinkIndex, targetPosition, targetOrientation)
    ccall((:b3CalculateInverseKinematicsAddTargetPositionWithOrientation, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, endEffectorLinkIndex, targetPosition, targetOrientation)
end

function b3CalculateInverseKinematicsPosWithNullSpaceVel(commandHandle, numDof, endEffectorLinkIndex, targetPosition, lowerLimit, upperLimit, jointRange, restPose)
    ccall((:b3CalculateInverseKinematicsPosWithNullSpaceVel, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, numDof, endEffectorLinkIndex, targetPosition, lowerLimit, upperLimit, jointRange, restPose)
end

function b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(commandHandle, numDof, endEffectorLinkIndex, targetPosition, targetOrientation, lowerLimit, upperLimit, jointRange, restPose)
    ccall((:b3CalculateInverseKinematicsPosOrnWithNullSpaceVel, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, numDof, endEffectorLinkIndex, targetPosition, targetOrientation, lowerLimit, upperLimit, jointRange, restPose)
end

function b3CalculateInverseKinematicsSetJointDamping(commandHandle, numDof, jointDampingCoeff)
    ccall((:b3CalculateInverseKinematicsSetJointDamping, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), commandHandle, numDof, jointDampingCoeff)
end

function b3CalculateInverseKinematicsSelectSolver(commandHandle, solver)
    ccall((:b3CalculateInverseKinematicsSelectSolver, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, solver)
end

function b3GetStatusInverseKinematicsJointPositions(statusHandle, bodyUniqueId, dofCount, jointPositions)
    ccall((:b3GetStatusInverseKinematicsJointPositions, libclang), Cint, (b3SharedMemoryStatusHandle, Ptr{Cint}, Ptr{Cint}, Ptr{Cdouble}), statusHandle, bodyUniqueId, dofCount, jointPositions)
end

function b3CalculateInverseKinematicsSetCurrentPositions(commandHandle, numDof, currentJointPositions)
    ccall((:b3CalculateInverseKinematicsSetCurrentPositions, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), commandHandle, numDof, currentJointPositions)
end

function b3CalculateInverseKinematicsSetMaxNumIterations(commandHandle, maxNumIterations)
    ccall((:b3CalculateInverseKinematicsSetMaxNumIterations, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, maxNumIterations)
end

function b3CalculateInverseKinematicsSetResidualThreshold(commandHandle, residualThreshold)
    ccall((:b3CalculateInverseKinematicsSetResidualThreshold, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, residualThreshold)
end

function b3CollisionFilterCommandInit(physClient)
    ccall((:b3CollisionFilterCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3SetCollisionFilterPair(commandHandle, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, enableCollision)
    ccall((:b3SetCollisionFilterPair, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Cint, Cint, Cint), commandHandle, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, enableCollision)
end

function b3SetCollisionFilterGroupMask(commandHandle, bodyUniqueIdA, linkIndexA, collisionFilterGroup, collisionFilterMask)
    ccall((:b3SetCollisionFilterGroupMask, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Cint, Cint), commandHandle, bodyUniqueIdA, linkIndexA, collisionFilterGroup, collisionFilterMask)
end

function b3LoadSdfCommandInit(physClient, sdfFileName)
    ccall((:b3LoadSdfCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, sdfFileName)
end

function b3LoadSdfCommandInit2(commandHandle, sdfFileName)
    ccall((:b3LoadSdfCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cstring), commandHandle, sdfFileName)
end

function b3LoadSdfCommandSetUseMultiBody(commandHandle, useMultiBody)
    ccall((:b3LoadSdfCommandSetUseMultiBody, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, useMultiBody)
end

function b3LoadSdfCommandSetUseGlobalScaling(commandHandle, globalScaling)
    ccall((:b3LoadSdfCommandSetUseGlobalScaling, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, globalScaling)
end

function b3SaveWorldCommandInit(physClient, sdfFileName)
    ccall((:b3SaveWorldCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, sdfFileName)
end

function b3JointControlCommandInit(physClient, controlMode)
    ccall((:b3JointControlCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, controlMode)
end

function b3JointControlCommandInit2(physClient, bodyUniqueId, controlMode)
    ccall((:b3JointControlCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint, Cint), physClient, bodyUniqueId, controlMode)
end

function b3JointControlCommandInit2Internal(commandHandle, bodyUniqueId, controlMode)
    ccall((:b3JointControlCommandInit2Internal, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, bodyUniqueId, controlMode)
end

function b3JointControlSetDesiredPosition(commandHandle, qIndex, value)
    ccall((:b3JointControlSetDesiredPosition, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, qIndex, value)
end

function b3JointControlSetDesiredPositionMultiDof(commandHandle, qIndex, position, dofCount)
    ccall((:b3JointControlSetDesiredPositionMultiDof, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Cint), commandHandle, qIndex, position, dofCount)
end

function b3JointControlSetKp(commandHandle, dofIndex, value)
    ccall((:b3JointControlSetKp, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, dofIndex, value)
end

function b3JointControlSetKd(commandHandle, dofIndex, value)
    ccall((:b3JointControlSetKd, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, dofIndex, value)
end

function b3JointControlSetMaximumVelocity(commandHandle, dofIndex, maximumVelocity)
    ccall((:b3JointControlSetMaximumVelocity, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, dofIndex, maximumVelocity)
end

function b3JointControlSetDesiredVelocity(commandHandle, dofIndex, value)
    ccall((:b3JointControlSetDesiredVelocity, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, dofIndex, value)
end

function b3JointControlSetDesiredVelocityMultiDof(commandHandle, dofIndex, velocity, dofCount)
    ccall((:b3JointControlSetDesiredVelocityMultiDof, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Cint), commandHandle, dofIndex, velocity, dofCount)
end

function b3JointControlSetMaximumForce(commandHandle, dofIndex, value)
    ccall((:b3JointControlSetMaximumForce, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, dofIndex, value)
end

function b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, dofIndex, forces, dofCount)
    ccall((:b3JointControlSetDesiredForceTorqueMultiDof, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Cint), commandHandle, dofIndex, forces, dofCount)
end

function b3JointControlSetDesiredForceTorque(commandHandle, dofIndex, value)
    ccall((:b3JointControlSetDesiredForceTorque, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cdouble), commandHandle, dofIndex, value)
end

function b3CreateCollisionShapeCommandInit(physClient)
    ccall((:b3CreateCollisionShapeCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3CreateCollisionShapeAddSphere(commandHandle, radius)
    ccall((:b3CreateCollisionShapeAddSphere, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, radius)
end

function b3CreateCollisionShapeAddBox(commandHandle, halfExtents)
    ccall((:b3CreateCollisionShapeAddBox, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, halfExtents)
end

function b3CreateCollisionShapeAddCapsule(commandHandle, radius, height)
    ccall((:b3CreateCollisionShapeAddCapsule, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble), commandHandle, radius, height)
end

function b3CreateCollisionShapeAddCylinder(commandHandle, radius, height)
    ccall((:b3CreateCollisionShapeAddCylinder, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble), commandHandle, radius, height)
end

function b3CreateCollisionShapeAddPlane(commandHandle, planeNormal, planeConstant)
    ccall((:b3CreateCollisionShapeAddPlane, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}, Cdouble), commandHandle, planeNormal, planeConstant)
end

function b3CreateCollisionShapeAddMesh(commandHandle, fileName, meshScale)
    ccall((:b3CreateCollisionShapeAddMesh, libclang), Cint, (b3SharedMemoryCommandHandle, Cstring, Ptr{Cdouble}), commandHandle, fileName, meshScale)
end

function b3CreateCollisionShapeAddConvexMesh(physClient, commandHandle, meshScale, vertices, numVertices)
    ccall((:b3CreateCollisionShapeAddConvexMesh, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Ptr{Cdouble}, Ptr{Cdouble}, Cint), physClient, commandHandle, meshScale, vertices, numVertices)
end

function b3CreateCollisionShapeAddConcaveMesh(physClient, commandHandle, meshScale, vertices, numVertices, indices, numIndices)
    ccall((:b3CreateCollisionShapeAddConcaveMesh, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Ptr{Cdouble}, Ptr{Cdouble}, Cint, Ptr{Cint}, Cint), physClient, commandHandle, meshScale, vertices, numVertices, indices, numIndices)
end

function b3CreateCollisionSetFlag(commandHandle, shapeIndex, flags)
    ccall((:b3CreateCollisionSetFlag, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, shapeIndex, flags)
end

function b3CreateCollisionShapeSetChildTransform(commandHandle, shapeIndex, childPosition, childOrientation)
    ccall((:b3CreateCollisionShapeSetChildTransform, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, shapeIndex, childPosition, childOrientation)
end

function b3GetStatusCollisionShapeUniqueId(statusHandle)
    ccall((:b3GetStatusCollisionShapeUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3InitRemoveCollisionShapeCommand(physClient, collisionShapeId)
    ccall((:b3InitRemoveCollisionShapeCommand, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, collisionShapeId)
end

function b3CreateVisualShapeCommandInit(physClient)
    ccall((:b3CreateVisualShapeCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3CreateVisualShapeAddSphere(commandHandle, radius)
    ccall((:b3CreateVisualShapeAddSphere, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, radius)
end

function b3CreateVisualShapeAddBox(commandHandle, halfExtents)
    ccall((:b3CreateVisualShapeAddBox, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, halfExtents)
end

function b3CreateVisualShapeAddCapsule(commandHandle, radius, height)
    ccall((:b3CreateVisualShapeAddCapsule, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble), commandHandle, radius, height)
end

function b3CreateVisualShapeAddCylinder(commandHandle, radius, height)
    ccall((:b3CreateVisualShapeAddCylinder, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble), commandHandle, radius, height)
end

function b3CreateVisualShapeAddPlane(commandHandle, planeNormal, planeConstant)
    ccall((:b3CreateVisualShapeAddPlane, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}, Cdouble), commandHandle, planeNormal, planeConstant)
end

function b3CreateVisualShapeAddMesh(commandHandle, fileName, meshScale)
    ccall((:b3CreateVisualShapeAddMesh, libclang), Cint, (b3SharedMemoryCommandHandle, Cstring, Ptr{Cdouble}), commandHandle, fileName, meshScale)
end

function b3CreateVisualShapeAddMesh2(physClient, commandHandle, meshScale, vertices, numVertices, indices, numIndices, normals, numNormals, uvs, numUVs)
    ccall((:b3CreateVisualShapeAddMesh2, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Ptr{Cdouble}, Ptr{Cdouble}, Cint, Ptr{Cint}, Cint, Ptr{Cdouble}, Cint, Ptr{Cdouble}, Cint), physClient, commandHandle, meshScale, vertices, numVertices, indices, numIndices, normals, numNormals, uvs, numUVs)
end

function b3CreateVisualSetFlag(commandHandle, shapeIndex, flags)
    ccall((:b3CreateVisualSetFlag, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, shapeIndex, flags)
end

function b3CreateVisualShapeSetChildTransform(commandHandle, shapeIndex, childPosition, childOrientation)
    ccall((:b3CreateVisualShapeSetChildTransform, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, shapeIndex, childPosition, childOrientation)
end

function b3CreateVisualShapeSetSpecularColor(commandHandle, shapeIndex, specularColor)
    ccall((:b3CreateVisualShapeSetSpecularColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), commandHandle, shapeIndex, specularColor)
end

function b3CreateVisualShapeSetRGBAColor(commandHandle, shapeIndex, rgbaColor)
    ccall((:b3CreateVisualShapeSetRGBAColor, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), commandHandle, shapeIndex, rgbaColor)
end

function b3GetStatusVisualShapeUniqueId(statusHandle)
    ccall((:b3GetStatusVisualShapeUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3CreateMultiBodyCommandInit(physClient)
    ccall((:b3CreateMultiBodyCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3CreateMultiBodyBase(commandHandle, mass, collisionShapeUnique, visualShapeUniqueId, basePosition, baseOrientation, baseInertialFramePosition, baseInertialFrameOrientation)
    ccall((:b3CreateMultiBodyBase, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cint, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, mass, collisionShapeUnique, visualShapeUniqueId, basePosition, baseOrientation, baseInertialFramePosition, baseInertialFrameOrientation)
end

function b3CreateMultiBodyLink(commandHandle, linkMass, linkCollisionShapeIndex, linkVisualShapeIndex, linkPosition, linkOrientation, linkInertialFramePosition, linkInertialFrameOrientation, linkParentIndex, linkJointType, linkJointAxis)
    ccall((:b3CreateMultiBodyLink, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Cint, Cint, Ptr{Cdouble}), commandHandle, linkMass, linkCollisionShapeIndex, linkVisualShapeIndex, linkPosition, linkOrientation, linkInertialFramePosition, linkInertialFrameOrientation, linkParentIndex, linkJointType, linkJointAxis)
end

function b3CreateMultiBodySetBatchPositions(physClient, commandHandle, batchPositions, numBatchObjects)
    ccall((:b3CreateMultiBodySetBatchPositions, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Ptr{Cdouble}, Cint), physClient, commandHandle, batchPositions, numBatchObjects)
end

function b3CreateMultiBodyUseMaximalCoordinates(commandHandle)
    ccall((:b3CreateMultiBodyUseMaximalCoordinates, libclang), Cvoid, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3CreateMultiBodySetFlags(commandHandle, flags)
    ccall((:b3CreateMultiBodySetFlags, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, flags)
end

function b3CreateBoxShapeCommandInit(physClient)
    ccall((:b3CreateBoxShapeCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3CreateBoxCommandSetStartPosition(commandHandle, startPosX, startPosY, startPosZ)
    ccall((:b3CreateBoxCommandSetStartPosition, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble), commandHandle, startPosX, startPosY, startPosZ)
end

function b3CreateBoxCommandSetStartOrientation(commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
    ccall((:b3CreateBoxCommandSetStartOrientation, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble, Cdouble), commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
end

function b3CreateBoxCommandSetHalfExtents(commandHandle, halfExtentsX, halfExtentsY, halfExtentsZ)
    ccall((:b3CreateBoxCommandSetHalfExtents, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble), commandHandle, halfExtentsX, halfExtentsY, halfExtentsZ)
end

function b3CreateBoxCommandSetMass(commandHandle, mass)
    ccall((:b3CreateBoxCommandSetMass, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, mass)
end

function b3CreateBoxCommandSetCollisionShapeType(commandHandle, collisionShapeType)
    ccall((:b3CreateBoxCommandSetCollisionShapeType, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, collisionShapeType)
end

function b3CreateBoxCommandSetColorRGBA(commandHandle, red, green, blue, alpha)
    ccall((:b3CreateBoxCommandSetColorRGBA, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble, Cdouble), commandHandle, red, green, blue, alpha)
end

function b3CreatePoseCommandInit(physClient, bodyUniqueId)
    ccall((:b3CreatePoseCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3CreatePoseCommandInit2(commandHandle, bodyUniqueId)
    ccall((:b3CreatePoseCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyUniqueId)
end

function b3CreatePoseCommandSetBasePosition(commandHandle, startPosX, startPosY, startPosZ)
    ccall((:b3CreatePoseCommandSetBasePosition, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble), commandHandle, startPosX, startPosY, startPosZ)
end

function b3CreatePoseCommandSetBaseOrientation(commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
    ccall((:b3CreatePoseCommandSetBaseOrientation, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble, Cdouble, Cdouble, Cdouble), commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
end

function b3CreatePoseCommandSetBaseLinearVelocity(commandHandle, linVel)
    ccall((:b3CreatePoseCommandSetBaseLinearVelocity, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, linVel)
end

function b3CreatePoseCommandSetBaseAngularVelocity(commandHandle, angVel)
    ccall((:b3CreatePoseCommandSetBaseAngularVelocity, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, angVel)
end

function b3CreatePoseCommandSetJointPositions(commandHandle, numJointPositions, jointPositions)
    ccall((:b3CreatePoseCommandSetJointPositions, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), commandHandle, numJointPositions, jointPositions)
end

function b3CreatePoseCommandSetJointPosition(physClient, commandHandle, jointIndex, jointPosition)
    ccall((:b3CreatePoseCommandSetJointPosition, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Cint, Cdouble), physClient, commandHandle, jointIndex, jointPosition)
end

function b3CreatePoseCommandSetJointPositionMultiDof(physClient, commandHandle, jointIndex, jointPosition, posSize)
    ccall((:b3CreatePoseCommandSetJointPositionMultiDof, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Cint), physClient, commandHandle, jointIndex, jointPosition, posSize)
end

function b3CreatePoseCommandSetQ(commandHandle, numJointPositions, q, hasQ)
    ccall((:b3CreatePoseCommandSetQ, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Ptr{Cint}), commandHandle, numJointPositions, q, hasQ)
end

function b3CreatePoseCommandSetQdots(commandHandle, numJointVelocities, qDots, hasQdots)
    ccall((:b3CreatePoseCommandSetQdots, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Ptr{Cint}), commandHandle, numJointVelocities, qDots, hasQdots)
end

function b3CreatePoseCommandSetJointVelocities(physClient, commandHandle, numJointVelocities, jointVelocities)
    ccall((:b3CreatePoseCommandSetJointVelocities, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}), physClient, commandHandle, numJointVelocities, jointVelocities)
end

function b3CreatePoseCommandSetJointVelocity(physClient, commandHandle, jointIndex, jointVelocity)
    ccall((:b3CreatePoseCommandSetJointVelocity, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Cint, Cdouble), physClient, commandHandle, jointIndex, jointVelocity)
end

function b3CreatePoseCommandSetJointVelocityMultiDof(physClient, commandHandle, jointIndex, jointVelocity, velSize)
    ccall((:b3CreatePoseCommandSetJointVelocityMultiDof, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Cint, Ptr{Cdouble}, Cint), physClient, commandHandle, jointIndex, jointVelocity, velSize)
end

function b3CreateSensorCommandInit(physClient, bodyUniqueId)
    ccall((:b3CreateSensorCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3CreateSensorEnable6DofJointForceTorqueSensor(commandHandle, jointIndex, enable)
    ccall((:b3CreateSensorEnable6DofJointForceTorqueSensor, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, jointIndex, enable)
end

function b3CreateSensorEnableIMUForLink(commandHandle, linkIndex, enable)
    ccall((:b3CreateSensorEnableIMUForLink, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, linkIndex, enable)
end

function b3RequestActualStateCommandInit(physClient, bodyUniqueId)
    ccall((:b3RequestActualStateCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cint), physClient, bodyUniqueId)
end

function b3RequestActualStateCommandInit2(commandHandle, bodyUniqueId)
    ccall((:b3RequestActualStateCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyUniqueId)
end

function b3RequestActualStateCommandComputeLinkVelocity(commandHandle, computeLinkVelocity)
    ccall((:b3RequestActualStateCommandComputeLinkVelocity, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, computeLinkVelocity)
end

function b3RequestActualStateCommandComputeForwardKinematics(commandHandle, computeForwardKinematics)
    ccall((:b3RequestActualStateCommandComputeForwardKinematics, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, computeForwardKinematics)
end

function b3GetJointState(physClient, statusHandle, jointIndex, state)
    ccall((:b3GetJointState, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryStatusHandle, Cint, Ptr{b3JointSensorState}), physClient, statusHandle, jointIndex, state)
end

function b3GetJointStateMultiDof(physClient, statusHandle, jointIndex, state)
    ccall((:b3GetJointStateMultiDof, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryStatusHandle, Cint, Ptr{b3JointSensorState2}), physClient, statusHandle, jointIndex, state)
end

function b3GetLinkState(physClient, statusHandle, linkIndex, state)
    ccall((:b3GetLinkState, libclang), Cint, (b3PhysicsClientHandle, b3SharedMemoryStatusHandle, Cint, Ptr{b3LinkState}), physClient, statusHandle, linkIndex, state)
end

function b3PickBody(physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
    ccall((:b3PickBody, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble), physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
end

function b3MovePickedBody(physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
    ccall((:b3MovePickedBody, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble), physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
end

function b3RemovePickingConstraint(physClient)
    ccall((:b3RemovePickingConstraint, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3CreateRaycastCommandInit(physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
    ccall((:b3CreateRaycastCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble, Cdouble), physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
end

function b3CreateRaycastBatchCommandInit(physClient)
    ccall((:b3CreateRaycastBatchCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3RaycastBatchSetNumThreads(commandHandle, numThreads)
    ccall((:b3RaycastBatchSetNumThreads, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, numThreads)
end

function b3RaycastBatchAddRay(commandHandle, rayFromWorld, rayToWorld)
    ccall((:b3RaycastBatchAddRay, libclang), Cvoid, (b3SharedMemoryCommandHandle, Ptr{Cdouble}, Ptr{Cdouble}), commandHandle, rayFromWorld, rayToWorld)
end

function b3RaycastBatchAddRays(physClient, commandHandle, rayFromWorld, rayToWorld, numRays)
    ccall((:b3RaycastBatchAddRays, libclang), Cvoid, (b3PhysicsClientHandle, b3SharedMemoryCommandHandle, Ptr{Cdouble}, Ptr{Cdouble}, Cint), physClient, commandHandle, rayFromWorld, rayToWorld, numRays)
end

function b3RaycastBatchSetParentObject(commandHandle, parentObjectUniqueId, parentLinkIndex)
    ccall((:b3RaycastBatchSetParentObject, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint), commandHandle, parentObjectUniqueId, parentLinkIndex)
end

function b3GetRaycastInformation(physClient, raycastInfo)
    ccall((:b3GetRaycastInformation, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3RaycastInformation}), physClient, raycastInfo)
end

function b3ApplyExternalForceCommandInit(physClient)
    ccall((:b3ApplyExternalForceCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3ApplyExternalForce(commandHandle, bodyUniqueId, linkId, force, position, flag)
    ccall((:b3ApplyExternalForce, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}, Ptr{Cdouble}, Cint), commandHandle, bodyUniqueId, linkId, force, position, flag)
end

function b3ApplyExternalTorque(commandHandle, bodyUniqueId, linkId, torque, flag)
    ccall((:b3ApplyExternalTorque, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint, Cint, Ptr{Cdouble}, Cint), commandHandle, bodyUniqueId, linkId, torque, flag)
end

function b3LoadSoftBodyCommandInit(physClient, fileName)
    ccall((:b3LoadSoftBodyCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, fileName)
end

function b3LoadSoftBodySetScale(commandHandle, scale)
    ccall((:b3LoadSoftBodySetScale, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, scale)
end

function b3LoadSoftBodySetMass(commandHandle, mass)
    ccall((:b3LoadSoftBodySetMass, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, mass)
end

function b3LoadSoftBodySetCollisionMargin(commandHandle, collisionMargin)
    ccall((:b3LoadSoftBodySetCollisionMargin, libclang), Cint, (b3SharedMemoryCommandHandle, Cdouble), commandHandle, collisionMargin)
end

function b3RequestVREventsCommandInit(physClient)
    ccall((:b3RequestVREventsCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3VREventsSetDeviceTypeFilter(commandHandle, deviceTypeFilter)
    ccall((:b3VREventsSetDeviceTypeFilter, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, deviceTypeFilter)
end

function b3GetVREventsData(physClient, vrEventsData)
    ccall((:b3GetVREventsData, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3VREventsData}), physClient, vrEventsData)
end

function b3SetVRCameraStateCommandInit(physClient)
    ccall((:b3SetVRCameraStateCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3SetVRCameraRootPosition(commandHandle, rootPos)
    ccall((:b3SetVRCameraRootPosition, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, rootPos)
end

function b3SetVRCameraRootOrientation(commandHandle, rootOrn)
    ccall((:b3SetVRCameraRootOrientation, libclang), Cint, (b3SharedMemoryCommandHandle, Ptr{Cdouble}), commandHandle, rootOrn)
end

function b3SetVRCameraTrackingObject(commandHandle, objectUniqueId)
    ccall((:b3SetVRCameraTrackingObject, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, objectUniqueId)
end

function b3SetVRCameraTrackingObjectFlag(commandHandle, flag)
    ccall((:b3SetVRCameraTrackingObjectFlag, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, flag)
end

function b3RequestKeyboardEventsCommandInit(physClient)
    ccall((:b3RequestKeyboardEventsCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3RequestKeyboardEventsCommandInit2(commandHandle)
    ccall((:b3RequestKeyboardEventsCommandInit2, libclang), b3SharedMemoryCommandHandle, (b3SharedMemoryCommandHandle,), commandHandle)
end

function b3GetKeyboardEventsData(physClient, keyboardEventsData)
    ccall((:b3GetKeyboardEventsData, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3KeyboardEventsData}), physClient, keyboardEventsData)
end

function b3RequestMouseEventsCommandInit(physClient)
    ccall((:b3RequestMouseEventsCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3GetMouseEventsData(physClient, mouseEventsData)
    ccall((:b3GetMouseEventsData, libclang), Cvoid, (b3PhysicsClientHandle, Ptr{b3MouseEventsData}), physClient, mouseEventsData)
end

function b3StateLoggingCommandInit(physClient)
    ccall((:b3StateLoggingCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle,), physClient)
end

function b3StateLoggingStart(commandHandle, loggingType, fileName)
    ccall((:b3StateLoggingStart, libclang), Cint, (b3SharedMemoryCommandHandle, Cint, Cstring), commandHandle, loggingType, fileName)
end

function b3StateLoggingAddLoggingObjectUniqueId(commandHandle, objectUniqueId)
    ccall((:b3StateLoggingAddLoggingObjectUniqueId, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, objectUniqueId)
end

function b3StateLoggingSetMaxLogDof(commandHandle, maxLogDof)
    ccall((:b3StateLoggingSetMaxLogDof, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, maxLogDof)
end

function b3StateLoggingSetLinkIndexA(commandHandle, linkIndexA)
    ccall((:b3StateLoggingSetLinkIndexA, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, linkIndexA)
end

function b3StateLoggingSetLinkIndexB(commandHandle, linkIndexB)
    ccall((:b3StateLoggingSetLinkIndexB, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, linkIndexB)
end

function b3StateLoggingSetBodyAUniqueId(commandHandle, bodyAUniqueId)
    ccall((:b3StateLoggingSetBodyAUniqueId, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyAUniqueId)
end

function b3StateLoggingSetBodyBUniqueId(commandHandle, bodyBUniqueId)
    ccall((:b3StateLoggingSetBodyBUniqueId, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, bodyBUniqueId)
end

function b3StateLoggingSetDeviceTypeFilter(commandHandle, deviceTypeFilter)
    ccall((:b3StateLoggingSetDeviceTypeFilter, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, deviceTypeFilter)
end

function b3StateLoggingSetLogFlags(commandHandle, logFlags)
    ccall((:b3StateLoggingSetLogFlags, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, logFlags)
end

function b3GetStatusLoggingUniqueId(statusHandle)
    ccall((:b3GetStatusLoggingUniqueId, libclang), Cint, (b3SharedMemoryStatusHandle,), statusHandle)
end

function b3StateLoggingStop(commandHandle, loggingUid)
    ccall((:b3StateLoggingStop, libclang), Cint, (b3SharedMemoryCommandHandle, Cint), commandHandle, loggingUid)
end

function b3ProfileTimingCommandInit(physClient, name)
    ccall((:b3ProfileTimingCommandInit, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, name)
end

function b3SetProfileTimingDuractionInMicroSeconds(commandHandle, duration)
    ccall((:b3SetProfileTimingDuractionInMicroSeconds, libclang), Cvoid, (b3SharedMemoryCommandHandle, Cint), commandHandle, duration)
end

function b3PushProfileTiming(physClient, timingName)
    ccall((:b3PushProfileTiming, libclang), Cvoid, (b3PhysicsClientHandle, Cstring), physClient, timingName)
end

function b3PopProfileTiming(physClient)
    ccall((:b3PopProfileTiming, libclang), Cvoid, (b3PhysicsClientHandle,), physClient)
end

function b3SetTimeOut(physClient, timeOutInSeconds)
    ccall((:b3SetTimeOut, libclang), Cvoid, (b3PhysicsClientHandle, Cdouble), physClient, timeOutInSeconds)
end

function b3GetTimeOut(physClient)
    ccall((:b3GetTimeOut, libclang), Cdouble, (b3PhysicsClientHandle,), physClient)
end

function b3SetAdditionalSearchPath(physClient, path)
    ccall((:b3SetAdditionalSearchPath, libclang), b3SharedMemoryCommandHandle, (b3PhysicsClientHandle, Cstring), physClient, path)
end

function b3MultiplyTransforms(posA, ornA, posB, ornB, outPos, outOrn)
    ccall((:b3MultiplyTransforms, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), posA, ornA, posB, ornB, outPos, outOrn)
end

function b3InvertTransform(pos, orn, outPos, outOrn)
    ccall((:b3InvertTransform, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), pos, orn, outPos, outOrn)
end

function b3QuaternionSlerp(startQuat, endQuat, interpolationFraction, outOrn)
    ccall((:b3QuaternionSlerp, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Ptr{Cdouble}), startQuat, endQuat, interpolationFraction, outOrn)
end

function b3GetQuaternionFromAxisAngle(axis, angle, outQuat)
    ccall((:b3GetQuaternionFromAxisAngle, libclang), Cvoid, (Ptr{Cdouble}, Cdouble, Ptr{Cdouble}), axis, angle, outQuat)
end

function b3GetAxisAngleFromQuaternion(quat, axis, angle)
    ccall((:b3GetAxisAngleFromQuaternion, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), quat, axis, angle)
end

function b3GetQuaternionDifference(startQuat, endQuat, outOrn)
    ccall((:b3GetQuaternionDifference, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), startQuat, endQuat, outOrn)
end

function b3GetAxisDifferenceQuaternion(startQuat, endQuat, axisOut)
    ccall((:b3GetAxisDifferenceQuaternion, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), startQuat, endQuat, axisOut)
end

function b3CalculateVelocityQuaternion(startQuat, endQuat, deltaTime, angVelOut)
    ccall((:b3CalculateVelocityQuaternion, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Cdouble, Ptr{Cdouble}), startQuat, endQuat, deltaTime, angVelOut)
end

function b3RotateVector(quat, vec, vecOut)
    ccall((:b3RotateVector, libclang), Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}), quat, vec, vecOut)
end
# Julia wrapper for header: SharedMemoryPublic.h
# Automatically generated using Clang.jl

# Julia wrapper for header: PhysicsDirectC_API.h
# Automatically generated using Clang.jl


function b3ConnectPhysicsDirect()
    ccall((:b3ConnectPhysicsDirect, libclang), b3PhysicsClientHandle, ())
end
# Julia wrapper for header: SharedMemoryInProcessPhysicsC_API.h
# Automatically generated using Clang.jl


function b3CreateInProcessPhysicsServerAndConnect(argc, argv)
    ccall((:b3CreateInProcessPhysicsServerAndConnect, libclang), b3PhysicsClientHandle, (Cint, Ptr{Cstring}), argc, argv)
end

function b3CreateInProcessPhysicsServerAndConnectSharedMemory(argc, argv)
    ccall((:b3CreateInProcessPhysicsServerAndConnectSharedMemory, libclang), b3PhysicsClientHandle, (Cint, Ptr{Cstring}), argc, argv)
end

function b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv)
    ccall((:b3CreateInProcessPhysicsServerAndConnectMainThread, libclang), b3PhysicsClientHandle, (Cint, Ptr{Cstring}), argc, argv)
end

function b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(argc, argv)
    ccall((:b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory, libclang), b3PhysicsClientHandle, (Cint, Ptr{Cstring}), argc, argv)
end

function b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(guiHelperPtr)
    ccall((:b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect, libclang), b3PhysicsClientHandle, (Ptr{Cvoid},), guiHelperPtr)
end

function b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect2(guiHelperPtr)
    ccall((:b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect2, libclang), b3PhysicsClientHandle, (Ptr{Cvoid},), guiHelperPtr)
end

function b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3(guiHelperPtr, sharedMemoryKey)
    ccall((:b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3, libclang), b3PhysicsClientHandle, (Ptr{Cvoid}, Cint), guiHelperPtr, sharedMemoryKey)
end

function b3InProcessRenderSceneInternal(clientHandle)
    ccall((:b3InProcessRenderSceneInternal, libclang), Cvoid, (b3PhysicsClientHandle,), clientHandle)
end

function b3InProcessDebugDrawInternal(clientHandle, debugDrawMode)
    ccall((:b3InProcessDebugDrawInternal, libclang), Cvoid, (b3PhysicsClientHandle, Cint), clientHandle, debugDrawMode)
end

function b3InProcessMouseMoveCallback(clientHandle, x, y)
    ccall((:b3InProcessMouseMoveCallback, libclang), Cint, (b3PhysicsClientHandle, Cfloat, Cfloat), clientHandle, x, y)
end

function b3InProcessMouseButtonCallback(clientHandle, button, state, x, y)
    ccall((:b3InProcessMouseButtonCallback, libclang), Cint, (b3PhysicsClientHandle, Cint, Cint, Cfloat, Cfloat), clientHandle, button, state, x, y)
end
