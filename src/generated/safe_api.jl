#AAutogeneratd by: src/generator/make_julian_api.jl
module Safe
import ..Raw
using ..Julian
function InitUserDebugDrawAddLine3D(physClient, fromXYZ, toXYZ, color, lineWidth, lifeTime)
    fromXYZ = bullet_vector3(Cdouble, fromXYZ)
    toXYZ = bullet_vector3(Cdouble, toXYZ)
    colorRGB = bullet_color(Cdouble, color)
    Raw.b3InitUserDebugDrawAddLine3D(physClient, fromXYZ, toXYZ, colorRGB, lineWidth, lifeTime)
end
function InitUserDebugDrawAddText3D(physClient, txt, positionXYZ, color, textSize, lifeTime)
    positionXYZ = bullet_vector3(Cdouble, positionXYZ)
    colorRGB = bullet_color(Cdouble, color)
    Raw.b3InitUserDebugDrawAddText3D(physClient, txt, positionXYZ, colorRGB, textSize, lifeTime)
end
function SetDebugObjectColor(commandHandle, objectUniqueId, linkIndex, color)
    objectColorRGB = bullet_color(Cdouble, color)
    Raw.b3SetDebugObjectColor(commandHandle, objectUniqueId, linkIndex, objectColorRGB)
end
function RequestCameraImageSetLightColor(commandHandle, color)
    lightColor = bullet_color(Cfloat, color)
    Raw.b3RequestCameraImageSetLightColor(commandHandle, lightColor)
end
function UpdateVisualShapeRGBAColor(commandHandle, color)
    rgbaColor = bullet_color_alpha(Cdouble, color)
    Raw.b3UpdateVisualShapeRGBAColor(commandHandle, rgbaColor)
end
function UpdateVisualShapeSpecularColor(commandHandle, color)
    specularColor = bullet_color(Cdouble, color)
    Raw.b3UpdateVisualShapeSpecularColor(commandHandle, specularColor)
end
function CreateVisualShapeSetSpecularColor(commandHandle, shapeIndex, color)
    specularColor = bullet_color(Cdouble, color)
    Raw.b3CreateVisualShapeSetSpecularColor(commandHandle, shapeIndex, specularColor)
end
function CreateVisualShapeSetRGBAColor(commandHandle, shapeIndex, color)
    rgbaColor = bullet_color_alpha(Cdouble, color)
    Raw.b3CreateVisualShapeSetRGBAColor(commandHandle, shapeIndex, rgbaColor)
end
function InitChangeUserConstraintSetFrameInB(commandHandle, jointChildFrameOrn)
    jointChildFrameOrn = bullet_quaternion(Cdouble, jointChildFrameOrn)
    Raw.b3InitChangeUserConstraintSetFrameInB(commandHandle, jointChildFrameOrn)
end
function SetClosestDistanceFilterCollisionShapeOrientationA(commandHandle, collisionShapeOrientationA)
    collisionShapeOrientationA = bullet_quaternion(Cdouble, collisionShapeOrientationA)
    Raw.b3SetClosestDistanceFilterCollisionShapeOrientationA(commandHandle, collisionShapeOrientationA)
end
function SetClosestDistanceFilterCollisionShapeOrientationB(commandHandle, collisionShapeOrientationB)
    collisionShapeOrientationB = bullet_quaternion(Cdouble, collisionShapeOrientationB)
    Raw.b3SetClosestDistanceFilterCollisionShapeOrientationB(commandHandle, collisionShapeOrientationB)
end
function CalculateInverseKinematicsAddTargetPositionWithOrientation(commandHandle, endEffectorLinkIndex, targetPosition, targetOrientation)
    targetOrientation = bullet_quaternion(Cdouble, targetOrientation)
    Raw.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(commandHandle, endEffectorLinkIndex, targetPosition, targetOrientation)
end
function CalculateInverseKinematicsPosOrnWithNullSpaceVel(commandHandle, numDof, endEffectorLinkIndex, targetPosition, targetOrientation, lowerLimit, upperLimit, jointRange, restPose)
    targetOrientation = bullet_quaternion(Cdouble, targetOrientation)
    Raw.b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(commandHandle, numDof, endEffectorLinkIndex, targetPosition, targetOrientation, lowerLimit, upperLimit, jointRange, restPose)
end
function CreateCollisionShapeSetChildTransform(commandHandle, shapeIndex, childPosition, childOrientation)
    childPosition = bullet_vector3(Cdouble, childPosition)
    childOrientation = bullet_quaternion(Cdouble, childOrientation)
    Raw.b3CreateCollisionShapeSetChildTransform(commandHandle, shapeIndex, childPosition, childOrientation)
end
function CreateVisualShapeSetChildTransform(commandHandle, shapeIndex, childPosition, childOrientation)
    childPosition = bullet_vector3(Cdouble, childPosition)
    childOrientation = bullet_quaternion(Cdouble, childOrientation)
    Raw.b3CreateVisualShapeSetChildTransform(commandHandle, shapeIndex, childPosition, childOrientation)
end
function CreateMultiBodyBase(commandHandle, mass, collisionShapeUnique, visualShapeUniqueId, basePosition, baseOrientation, baseInertialFramePosition, baseInertialFrameOrientation)
    basePosition = bullet_vector3(Cdouble, basePosition)
    baseOrientation = bullet_quaternion(Cdouble, baseOrientation)
    baseInertialFramePosition = bullet_vector3(Cdouble, baseInertialFramePosition)
    baseInertialFrameOrientation = bullet_quaternion(Cdouble, baseInertialFrameOrientation)
    Raw.b3CreateMultiBodyBase(commandHandle, mass, collisionShapeUnique, visualShapeUniqueId, basePosition, baseOrientation, baseInertialFramePosition, baseInertialFrameOrientation)
end
function SetVRCameraRootOrientation(commandHandle, rootOrn)
    rootOrn = bullet_quaternion(Cdouble, rootOrn)
    Raw.b3SetVRCameraRootOrientation(commandHandle, rootOrn)
end
function MultiplyTransforms(posA, ornA, posB, ornB, outPos, outOrn)
    posA = bullet_vector3(Cdouble, posA)
    ornA = bullet_quaternion(Cdouble, ornA)
    posB = bullet_vector3(Cdouble, posB)
    ornB = bullet_quaternion(Cdouble, ornB)
    Raw.b3MultiplyTransforms(posA, ornA, posB, ornB, outPos, outOrn)
end
function InvertTransform(pos, orn, outPos, outOrn)
    orn = bullet_quaternion(Cdouble, orn)
    Raw.b3InvertTransform(pos, orn, outPos, outOrn)
end
function QuaternionSlerp(startQuat, endQuat, interpolationFraction, outOrn)
    startQuat = bullet_quaternion(Cdouble, startQuat)
    endQuat = bullet_quaternion(Cdouble, endQuat)
    Raw.b3QuaternionSlerp(startQuat, endQuat, interpolationFraction, outOrn)
end
function GetAxisAngleFromQuaternion(quat, axis, angle)
    quat = bullet_quaternion(Cdouble, quat)
    Raw.b3GetAxisAngleFromQuaternion(quat, axis, angle)
end
function GetQuaternionDifference(startQuat, endQuat, outOrn)
    startQuat = bullet_quaternion(Cdouble, startQuat)
    endQuat = bullet_quaternion(Cdouble, endQuat)
    Raw.b3GetQuaternionDifference(startQuat, endQuat, outOrn)
end
function GetAxisDifferenceQuaternion(startQuat, endQuat, axisOut)
    startQuat = bullet_quaternion(Cdouble, startQuat)
    endQuat = bullet_quaternion(Cdouble, endQuat)
    Raw.b3GetAxisDifferenceQuaternion(startQuat, endQuat, axisOut)
end
function CalculateVelocityQuaternion(startQuat, endQuat, deltaTime, angVelOut)
    startQuat = bullet_quaternion(Cdouble, startQuat)
    endQuat = bullet_quaternion(Cdouble, endQuat)
    Raw.b3CalculateVelocityQuaternion(startQuat, endQuat, deltaTime, angVelOut)
end
function RotateVector(quat, vec, vecOut)
    quat = bullet_quaternion(Cdouble, quat)
    vec = bullet_vector3(Cdouble, vec)
    Raw.b3RotateVector(quat, vec, vecOut)
end
function ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, cameraDistance, cameraPitch, cameraYaw, cameraTargetPosition)
    cameraTargetPosition = bullet_vector3(Cfloat, cameraTargetPosition)
    Raw.b3ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, cameraDistance, cameraPitch, cameraYaw, cameraTargetPosition)
end
function ComputeViewMatrixFromPositions(cameraPosition, cameraTargetPosition, cameraUp, viewMatrix)
    cameraPosition = bullet_vector3(Cfloat, cameraPosition)
    cameraTargetPosition = bullet_vector3(Cfloat, cameraTargetPosition)
    Raw.b3ComputeViewMatrixFromPositions(cameraPosition, cameraTargetPosition, cameraUp, viewMatrix)
end
function ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxis, viewMatrix)
    cameraTargetPosition = bullet_vector3(Cfloat, cameraTargetPosition)
    Raw.b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxis, viewMatrix)
end
function ComputePositionFromViewMatrix(viewMatrix, cameraPosition, cameraTargetPosition, cameraUp)
    cameraPosition = bullet_vector3(Cfloat, cameraPosition)
    cameraTargetPosition = bullet_vector3(Cfloat, cameraTargetPosition)
    Raw.b3ComputePositionFromViewMatrix(viewMatrix, cameraPosition, cameraTargetPosition, cameraUp)
end
function RequestCameraImageSetViewMatrix(commandHandle, cameraPosition, cameraTargetPosition, cameraUp)
    cameraPosition = bullet_vector3(Cfloat, cameraPosition)
    cameraTargetPosition = bullet_vector3(Cfloat, cameraTargetPosition)
    Raw.b3RequestCameraImageSetViewMatrix(commandHandle, cameraPosition, cameraTargetPosition, cameraUp)
end
function RequestCameraImageSetViewMatrix2(commandHandle, cameraTargetPosition, distance, yaw, pitch, roll, upAxis)
    cameraTargetPosition = bullet_vector3(Cfloat, cameraTargetPosition)
    Raw.b3RequestCameraImageSetViewMatrix2(commandHandle, cameraTargetPosition, distance, yaw, pitch, roll, upAxis)
end
function SetClosestDistanceFilterCollisionShapePositionA(commandHandle, collisionShapePositionA)
    collisionShapePositionA = bullet_vector3(Cdouble, collisionShapePositionA)
    Raw.b3SetClosestDistanceFilterCollisionShapePositionA(commandHandle, collisionShapePositionA)
end
function SetClosestDistanceFilterCollisionShapePositionB(commandHandle, collisionShapePositionB)
    collisionShapePositionB = bullet_vector3(Cdouble, collisionShapePositionB)
    Raw.b3SetClosestDistanceFilterCollisionShapePositionB(commandHandle, collisionShapePositionB)
end
function JointControlSetDesiredPositionMultiDof(commandHandle, qIndex, position, dofCount)
    position = bullet_vector3(Cdouble, position)
    Raw.b3JointControlSetDesiredPositionMultiDof(commandHandle, qIndex, position, dofCount)
end
function RaycastBatchAddRay(commandHandle, rayFromWorld, rayToWorld)
    rayFromWorld = bullet_vector3(Cdouble, rayFromWorld)
    rayToWorld = bullet_vector3(Cdouble, rayToWorld)
    Raw.b3RaycastBatchAddRay(commandHandle, rayFromWorld, rayToWorld)
end
function ApplyExternalForce(commandHandle, bodyUniqueId, linkId, force, position, flag)
    position = bullet_vector3(Cdouble, position)
    Raw.b3ApplyExternalForce(commandHandle, bodyUniqueId, linkId, force, position, flag)
end
function CreateBoxCommandSetColorRGBA(commandHandle, color)
    (red, green, blue, alpha) = bullet_color_alpha(Float64, color)
    Raw.b3CreateBoxCommandSetColorRGBA(commandHandle, red, green, blue, alpha)
end
function PhysicsParamSetGravity(commandHandle, grav)
    (gravx, gravy, gravz) = bullet_vector3(Float64, grav)
    Raw.b3PhysicsParamSetGravity(commandHandle, gravx, gravy, gravz)
end
function LoadUrdfCommandSetStartPosition(commandHandle, startPos)
    (startPosX, startPosY, startPosZ) = bullet_vector3(Float64, startPos)
    Raw.b3LoadUrdfCommandSetStartPosition(commandHandle, startPosX, startPosY, startPosZ)
end
function CreateBoxCommandSetStartPosition(commandHandle, startPos)
    (startPosX, startPosY, startPosZ) = bullet_vector3(Float64, startPos)
    Raw.b3CreateBoxCommandSetStartPosition(commandHandle, startPosX, startPosY, startPosZ)
end
function CreatePoseCommandSetBasePosition(commandHandle, startPos)
    (startPosX, startPosY, startPosZ) = bullet_vector3(Float64, startPos)
    Raw.b3CreatePoseCommandSetBasePosition(commandHandle, startPosX, startPosY, startPosZ)
end
function PickBody(physClient, rayFromWorld, rayToWorld)
    (rayFromWorldX, rayFromWorldY, rayFromWorldZ) = bullet_vector3(Float64, rayFromWorld)
    (rayToWorldX, rayToWorldY, rayToWorldZ) = bullet_vector3(Float64, rayToWorld)
    Raw.b3PickBody(physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
end
function MovePickedBody(physClient, rayFromWorld, rayToWorld)
    (rayFromWorldX, rayFromWorldY, rayFromWorldZ) = bullet_vector3(Float64, rayFromWorld)
    (rayToWorldX, rayToWorldY, rayToWorldZ) = bullet_vector3(Float64, rayToWorld)
    Raw.b3MovePickedBody(physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
end
function CreateRaycastCommandInit(physClient, rayFromWorld, rayToWorld)
    (rayFromWorldX, rayFromWorldY, rayFromWorldZ) = bullet_vector3(Float64, rayFromWorld)
    (rayToWorldX, rayToWorldY, rayToWorldZ) = bullet_vector3(Float64, rayToWorld)
    Raw.b3CreateRaycastCommandInit(physClient, rayFromWorldX, rayFromWorldY, rayFromWorldZ, rayToWorldX, rayToWorldY, rayToWorldZ)
end
function LoadUrdfCommandSetStartOrientation(commandHandle, startOrn)
    (startOrnX, startOrnY, startOrnZ, startOrnW) = bullet_quaternion(Float64, startOrn)
    Raw.b3LoadUrdfCommandSetStartOrientation(commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
end
function CreateBoxCommandSetStartOrientation(commandHandle, startOrn)
    (startOrnX, startOrnY, startOrnZ, startOrnW) = bullet_quaternion(Float64, startOrn)
    Raw.b3CreateBoxCommandSetStartOrientation(commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
end
function CreatePoseCommandSetBaseOrientation(commandHandle, startOrn)
    (startOrnX, startOrnY, startOrnZ, startOrnW) = bullet_quaternion(Float64, startOrn)
    Raw.b3CreatePoseCommandSetBaseOrientation(commandHandle, startOrnX, startOrnY, startOrnZ, startOrnW)
end
end
