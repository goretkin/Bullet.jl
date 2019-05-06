# Automatically generated using Clang.jl


# Skipping MacroDefinition: B3_DECLARE_HANDLE ( name ) typedef struct name ## __ { int unused ; } * name
# Skipping MacroDefinition: B3_SHARED_API __attribute__ ( ( visibility ( "default" ) ) )

struct b3PhysicsClientHandle__
    unused::Cint
end

const b3PhysicsClientHandle = Ptr{b3PhysicsClientHandle__}

struct b3SharedMemoryCommandHandle__
    unused::Cint
end

const b3SharedMemoryCommandHandle = Ptr{b3SharedMemoryCommandHandle__}

struct b3SharedMemoryStatusHandle__
    unused::Cint
end

const b3SharedMemoryStatusHandle = Ptr{b3SharedMemoryStatusHandle__}
const SHARED_MEMORY_KEY = 12347
const SHARED_MEMORY_MAGIC_NUMBER = 201904030
const MAX_VR_ANALOG_AXIS = 5
const MAX_VR_BUTTONS = 64
const MAX_VR_CONTROLLERS = 8
const MAX_KEYBOARD_EVENTS = 256
const MAX_MOUSE_EVENTS = 256
const MAX_SDF_BODIES = 512
const MAX_USER_DATA_KEY_LENGTH = 256
const MAX_RAY_INTERSECTION_BATCH_SIZE = 256
const MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING = 4 * 1024
const MAX_RAY_HITS = MAX_RAY_INTERSECTION_BATCH_SIZE
const VISUAL_SHAPE_MAX_PATH_LEN = 1024
const B3_MAX_PLUGIN_ARG_SIZE = 128
const B3_MAX_PLUGIN_ARG_TEXT_LEN = 1024
const MAX_ISLANDS_ANALYTICS = 1024
const B3_MAX_NUM_VERTICES = 8192
const B3_MAX_NUM_INDICES = 32768

@cenum(EnumSharedMemoryClientCommand,
    CMD_INVALID = 0,
    CMD_LOAD_SDF = 1,
    CMD_LOAD_URDF = 2,
    CMD_LOAD_BULLET = 3,
    CMD_SAVE_BULLET = 4,
    CMD_LOAD_MJCF = 5,
    CMD_LOAD_SOFT_BODY = 6,
    CMD_SEND_BULLET_DATA_STREAM = 7,
    CMD_CREATE_BOX_COLLISION_SHAPE = 8,
    CMD_CREATE_RIGID_BODY = 9,
    CMD_DELETE_RIGID_BODY = 10,
    CMD_CREATE_SENSOR = 11,
    CMD_INIT_POSE = 12,
    CMD_SEND_PHYSICS_SIMULATION_PARAMETERS = 13,
    CMD_SEND_DESIRED_STATE = 14,
    CMD_REQUEST_ACTUAL_STATE = 15,
    CMD_REQUEST_DEBUG_LINES = 16,
    CMD_REQUEST_BODY_INFO = 17,
    CMD_REQUEST_INTERNAL_DATA = 18,
    CMD_STEP_FORWARD_SIMULATION = 19,
    CMD_RESET_SIMULATION = 20,
    CMD_PICK_BODY = 21,
    CMD_MOVE_PICKED_BODY = 22,
    CMD_REMOVE_PICKING_CONSTRAINT_BODY = 23,
    CMD_REQUEST_CAMERA_IMAGE_DATA = 24,
    CMD_APPLY_EXTERNAL_FORCE = 25,
    CMD_CALCULATE_INVERSE_DYNAMICS = 26,
    CMD_CALCULATE_INVERSE_KINEMATICS = 27,
    CMD_CALCULATE_JACOBIAN = 28,
    CMD_CALCULATE_MASS_MATRIX = 29,
    CMD_USER_CONSTRAINT = 30,
    CMD_REQUEST_CONTACT_POINT_INFORMATION = 31,
    CMD_REQUEST_RAY_CAST_INTERSECTIONS = 32,
    CMD_REQUEST_AABB_OVERLAP = 33,
    CMD_SAVE_WORLD = 34,
    CMD_REQUEST_VISUAL_SHAPE_INFO = 35,
    CMD_UPDATE_VISUAL_SHAPE = 36,
    CMD_LOAD_TEXTURE = 37,
    CMD_SET_SHADOW = 38,
    CMD_USER_DEBUG_DRAW = 39,
    CMD_REQUEST_VR_EVENTS_DATA = 40,
    CMD_SET_VR_CAMERA_STATE = 41,
    CMD_SYNC_BODY_INFO = 42,
    CMD_STATE_LOGGING = 43,
    CMD_CONFIGURE_OPENGL_VISUALIZER = 44,
    CMD_REQUEST_KEYBOARD_EVENTS_DATA = 45,
    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA = 46,
    CMD_REMOVE_BODY = 47,
    CMD_CHANGE_DYNAMICS_INFO = 48,
    CMD_GET_DYNAMICS_INFO = 49,
    CMD_PROFILE_TIMING = 50,
    CMD_CREATE_COLLISION_SHAPE = 51,
    CMD_CREATE_VISUAL_SHAPE = 52,
    CMD_CREATE_MULTI_BODY = 53,
    CMD_REQUEST_COLLISION_INFO = 54,
    CMD_REQUEST_MOUSE_EVENTS_DATA = 55,
    CMD_CHANGE_TEXTURE = 56,
    CMD_SET_ADDITIONAL_SEARCH_PATH = 57,
    CMD_CUSTOM_COMMAND = 58,
    CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS = 59,
    CMD_SAVE_STATE = 60,
    CMD_RESTORE_STATE = 61,
    CMD_REMOVE_STATE = 62,
    CMD_REQUEST_COLLISION_SHAPE_INFO = 63,
    CMD_SYNC_USER_DATA = 64,
    CMD_REQUEST_USER_DATA = 65,
    CMD_ADD_USER_DATA = 66,
    CMD_REMOVE_USER_DATA = 67,
    CMD_COLLISION_FILTER = 68,
    CMD_MAX_CLIENT_COMMANDS = 69,
)
@cenum(EnumSharedMemoryServerStatus,
    CMD_SHARED_MEMORY_NOT_INITIALIZED = 0,
    CMD_WAITING_FOR_CLIENT_COMMAND = 1,
    CMD_CLIENT_COMMAND_COMPLETED = 2,
    CMD_UNKNOWN_COMMAND_FLUSHED = 3,
    CMD_SDF_LOADING_COMPLETED = 4,
    CMD_SDF_LOADING_FAILED = 5,
    CMD_URDF_LOADING_COMPLETED = 6,
    CMD_URDF_LOADING_FAILED = 7,
    CMD_BULLET_LOADING_COMPLETED = 8,
    CMD_BULLET_LOADING_FAILED = 9,
    CMD_BULLET_SAVING_COMPLETED = 10,
    CMD_BULLET_SAVING_FAILED = 11,
    CMD_MJCF_LOADING_COMPLETED = 12,
    CMD_MJCF_LOADING_FAILED = 13,
    CMD_REQUEST_INTERNAL_DATA_COMPLETED = 14,
    CMD_REQUEST_INTERNAL_DATA_FAILED = 15,
    CMD_BULLET_DATA_STREAM_RECEIVED_COMPLETED = 16,
    CMD_BULLET_DATA_STREAM_RECEIVED_FAILED = 17,
    CMD_BOX_COLLISION_SHAPE_CREATION_COMPLETED = 18,
    CMD_RIGID_BODY_CREATION_COMPLETED = 19,
    CMD_SET_JOINT_FEEDBACK_COMPLETED = 20,
    CMD_ACTUAL_STATE_UPDATE_COMPLETED = 21,
    CMD_ACTUAL_STATE_UPDATE_FAILED = 22,
    CMD_DEBUG_LINES_COMPLETED = 23,
    CMD_DEBUG_LINES_OVERFLOW_FAILED = 24,
    CMD_DESIRED_STATE_RECEIVED_COMPLETED = 25,
    CMD_STEP_FORWARD_SIMULATION_COMPLETED = 26,
    CMD_RESET_SIMULATION_COMPLETED = 27,
    CMD_CAMERA_IMAGE_COMPLETED = 28,
    CMD_CAMERA_IMAGE_FAILED = 29,
    CMD_BODY_INFO_COMPLETED = 30,
    CMD_BODY_INFO_FAILED = 31,
    CMD_INVALID_STATUS = 32,
    CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED = 33,
    CMD_CALCULATED_INVERSE_DYNAMICS_FAILED = 34,
    CMD_CALCULATED_JACOBIAN_COMPLETED = 35,
    CMD_CALCULATED_JACOBIAN_FAILED = 36,
    CMD_CALCULATED_MASS_MATRIX_COMPLETED = 37,
    CMD_CALCULATED_MASS_MATRIX_FAILED = 38,
    CMD_CONTACT_POINT_INFORMATION_COMPLETED = 39,
    CMD_CONTACT_POINT_INFORMATION_FAILED = 40,
    CMD_REQUEST_AABB_OVERLAP_COMPLETED = 41,
    CMD_REQUEST_AABB_OVERLAP_FAILED = 42,
    CMD_CALCULATE_INVERSE_KINEMATICS_COMPLETED = 43,
    CMD_CALCULATE_INVERSE_KINEMATICS_FAILED = 44,
    CMD_SAVE_WORLD_COMPLETED = 45,
    CMD_SAVE_WORLD_FAILED = 46,
    CMD_VISUAL_SHAPE_INFO_COMPLETED = 47,
    CMD_VISUAL_SHAPE_INFO_FAILED = 48,
    CMD_VISUAL_SHAPE_UPDATE_COMPLETED = 49,
    CMD_VISUAL_SHAPE_UPDATE_FAILED = 50,
    CMD_LOAD_TEXTURE_COMPLETED = 51,
    CMD_LOAD_TEXTURE_FAILED = 52,
    CMD_USER_DEBUG_DRAW_COMPLETED = 53,
    CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED = 54,
    CMD_USER_DEBUG_DRAW_FAILED = 55,
    CMD_USER_CONSTRAINT_COMPLETED = 56,
    CMD_USER_CONSTRAINT_INFO_COMPLETED = 57,
    CMD_USER_CONSTRAINT_REQUEST_STATE_COMPLETED = 58,
    CMD_REMOVE_USER_CONSTRAINT_COMPLETED = 59,
    CMD_CHANGE_USER_CONSTRAINT_COMPLETED = 60,
    CMD_REMOVE_USER_CONSTRAINT_FAILED = 61,
    CMD_CHANGE_USER_CONSTRAINT_FAILED = 62,
    CMD_USER_CONSTRAINT_FAILED = 63,
    CMD_REQUEST_VR_EVENTS_DATA_COMPLETED = 64,
    CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED = 65,
    CMD_SYNC_BODY_INFO_COMPLETED = 66,
    CMD_SYNC_BODY_INFO_FAILED = 67,
    CMD_STATE_LOGGING_COMPLETED = 68,
    CMD_STATE_LOGGING_START_COMPLETED = 69,
    CMD_STATE_LOGGING_FAILED = 70,
    CMD_REQUEST_KEYBOARD_EVENTS_DATA_COMPLETED = 71,
    CMD_REQUEST_KEYBOARD_EVENTS_DATA_FAILED = 72,
    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_FAILED = 73,
    CMD_REQUEST_OPENGL_VISUALIZER_CAMERA_COMPLETED = 74,
    CMD_REMOVE_BODY_COMPLETED = 75,
    CMD_REMOVE_BODY_FAILED = 76,
    CMD_GET_DYNAMICS_INFO_COMPLETED = 77,
    CMD_GET_DYNAMICS_INFO_FAILED = 78,
    CMD_CREATE_COLLISION_SHAPE_FAILED = 79,
    CMD_CREATE_COLLISION_SHAPE_COMPLETED = 80,
    CMD_CREATE_VISUAL_SHAPE_FAILED = 81,
    CMD_CREATE_VISUAL_SHAPE_COMPLETED = 82,
    CMD_CREATE_MULTI_BODY_FAILED = 83,
    CMD_CREATE_MULTI_BODY_COMPLETED = 84,
    CMD_REQUEST_COLLISION_INFO_COMPLETED = 85,
    CMD_REQUEST_COLLISION_INFO_FAILED = 86,
    CMD_REQUEST_MOUSE_EVENTS_DATA_COMPLETED = 87,
    CMD_CHANGE_TEXTURE_COMMAND_FAILED = 88,
    CMD_CUSTOM_COMMAND_COMPLETED = 89,
    CMD_CUSTOM_COMMAND_FAILED = 90,
    CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED = 91,
    CMD_SAVE_STATE_FAILED = 92,
    CMD_SAVE_STATE_COMPLETED = 93,
    CMD_RESTORE_STATE_FAILED = 94,
    CMD_RESTORE_STATE_COMPLETED = 95,
    CMD_COLLISION_SHAPE_INFO_COMPLETED = 96,
    CMD_COLLISION_SHAPE_INFO_FAILED = 97,
    CMD_LOAD_SOFT_BODY_FAILED = 98,
    CMD_LOAD_SOFT_BODY_COMPLETED = 99,
    CMD_SYNC_USER_DATA_COMPLETED = 100,
    CMD_SYNC_USER_DATA_FAILED = 101,
    CMD_REQUEST_USER_DATA_COMPLETED = 102,
    CMD_REQUEST_USER_DATA_FAILED = 103,
    CMD_ADD_USER_DATA_COMPLETED = 104,
    CMD_ADD_USER_DATA_FAILED = 105,
    CMD_REMOVE_USER_DATA_COMPLETED = 106,
    CMD_REMOVE_USER_DATA_FAILED = 107,
    CMD_REMOVE_STATE_COMPLETED = 108,
    CMD_REMOVE_STATE_FAILED = 109,
    CMD_MAX_SERVER_COMMANDS = 110,
)
@cenum(JointInfoFlags,
    JOINT_HAS_MOTORIZED_POWER = 1,
)
@cenum(JointType,
    eRevoluteType = 0,
    ePrismaticType = 1,
    eSphericalType = 2,
    ePlanarType = 3,
    eFixedType = 4,
    ePoint2PointType = 5,
    eGearType = 6,
)
@cenum(b3JointInfoFlags,
    eJointChangeMaxForce = 1,
    eJointChangeChildFramePosition = 2,
    eJointChangeChildFrameOrientation = 4,
)

struct b3JointInfo
    m_linkName::NTuple{1024, UInt8}
    m_jointName::NTuple{1024, UInt8}
    m_jointType::Cint
    m_qIndex::Cint
    m_uIndex::Cint
    m_jointIndex::Cint
    m_flags::Cint
    m_jointDamping::Cdouble
    m_jointFriction::Cdouble
    m_jointLowerLimit::Cdouble
    m_jointUpperLimit::Cdouble
    m_jointMaxForce::Cdouble
    m_jointMaxVelocity::Cdouble
    m_parentFrame::NTuple{7, Cdouble}
    m_childFrame::NTuple{7, Cdouble}
    m_jointAxis::NTuple{3, Cdouble}
    m_parentIndex::Cint
    m_qSize::Cint
    m_uSize::Cint
end

@cenum(UserDataValueType,
    USER_DATA_VALUE_TYPE_BYTES = 0,
    USER_DATA_VALUE_TYPE_STRING = 1,
)

struct b3UserDataValue
    m_type::Cint
    m_length::Cint
    m_data1::Cstring
end

struct b3UserConstraint
    m_parentBodyIndex::Cint
    m_parentJointIndex::Cint
    m_childBodyIndex::Cint
    m_childJointIndex::Cint
    m_parentFrame::NTuple{7, Cdouble}
    m_childFrame::NTuple{7, Cdouble}
    m_jointAxis::NTuple{3, Cdouble}
    m_jointType::Cint
    m_maxAppliedForce::Cdouble
    m_userConstraintUniqueId::Cint
    m_gearRatio::Cdouble
    m_gearAuxLink::Cint
    m_relativePositionTarget::Cdouble
    m_erp::Cdouble
end

struct b3BodyInfo
    m_baseName::NTuple{1024, UInt8}
    m_bodyName::NTuple{1024, UInt8}
end

@cenum(DynamicsActivationState,
    eActivationStateEnableSleeping = 1,
    eActivationStateDisableSleeping = 2,
    eActivationStateWakeUp = 4,
    eActivationStateSleep = 8,
    eActivationStateEnableWakeup = 16,
    eActivationStateDisableWakeup = 32,
)

struct b3DynamicsInfo
    m_mass::Cdouble
    m_localInertialDiagonal::NTuple{3, Cdouble}
    m_localInertialFrame::NTuple{7, Cdouble}
    m_lateralFrictionCoeff::Cdouble
    m_rollingFrictionCoeff::Cdouble
    m_spinningFrictionCoeff::Cdouble
    m_restitution::Cdouble
    m_contactStiffness::Cdouble
    m_contactDamping::Cdouble
    m_activationState::Cint
    m_angularDamping::Cdouble
    m_linearDamping::Cdouble
    m_ccdSweptSphereRadius::Cdouble
    m_contactProcessingThreshold::Cdouble
    m_frictionAnchor::Cint
end

@cenum(SensorType,
    eSensorForceTorqueType = 1,
)

struct b3JointSensorState
    m_jointPosition::Cdouble
    m_jointVelocity::Cdouble
    m_jointForceTorque::NTuple{6, Cdouble}
    m_jointMotorTorque::Cdouble
end

struct b3JointSensorState2
    m_jointPosition::NTuple{4, Cdouble}
    m_jointVelocity::NTuple{3, Cdouble}
    m_jointReactionForceTorque::NTuple{6, Cdouble}
    m_jointMotorTorqueMultiDof::NTuple{3, Cdouble}
    m_qDofSize::Cint
    m_uDofSize::Cint
end

struct b3DebugLines
    m_numDebugLines::Cint
    m_linesFrom::Ptr{Cfloat}
    m_linesTo::Ptr{Cfloat}
    m_linesColor::Ptr{Cfloat}
end

struct b3OverlappingObject
    m_objectUniqueId::Cint
    m_linkIndex::Cint
end

struct b3AABBOverlapData
    m_numOverlappingObjects::Cint
    m_overlappingObjects::Ptr{b3OverlappingObject}
end

struct b3CameraImageData
    m_pixelWidth::Cint
    m_pixelHeight::Cint
    m_rgbColorData::Ptr{Cuchar}
    m_depthValues::Ptr{Cfloat}
    m_segmentationMaskValues::Ptr{Cint}
end

struct b3OpenGLVisualizerCameraInfo
    m_width::Cint
    m_height::Cint
    m_viewMatrix::NTuple{16, Cfloat}
    m_projectionMatrix::NTuple{16, Cfloat}
    m_camUp::NTuple{3, Cfloat}
    m_camForward::NTuple{3, Cfloat}
    m_horizontal::NTuple{3, Cfloat}
    m_vertical::NTuple{3, Cfloat}
    m_yaw::Cfloat
    m_pitch::Cfloat
    m_dist::Cfloat
    m_target::NTuple{3, Cfloat}
end

struct b3UserConstraintState
    m_appliedConstraintForces::NTuple{6, Cdouble}
    m_numDofs::Cint
end

@cenum(b3VREventType,
    VR_CONTROLLER_MOVE_EVENT = 1,
    VR_CONTROLLER_BUTTON_EVENT = 2,
    VR_HMD_MOVE_EVENT = 4,
    VR_GENERIC_TRACKER_MOVE_EVENT = 8,
)
@cenum(b3VRButtonInfo,
    eButtonIsDown = 1,
    eButtonTriggered = 2,
    eButtonReleased = 4,
)
@cenum(eVRDeviceTypeEnums,
    VR_DEVICE_CONTROLLER = 1,
    VR_DEVICE_HMD = 2,
    VR_DEVICE_GENERIC_TRACKER = 4,
)
@cenum(EVRCameraFlags,
    VR_CAMERA_TRACK_OBJECT_ORIENTATION = 1,
)

struct b3VRControllerEvent
    m_controllerId::Cint
    m_deviceType::Cint
    m_numMoveEvents::Cint
    m_numButtonEvents::Cint
    m_pos::NTuple{4, Cfloat}
    m_orn::NTuple{4, Cfloat}
    m_analogAxis::Cfloat
    m_auxAnalogAxis::NTuple{10, Cfloat}
    m_buttons::NTuple{64, Cint}
end

struct b3VREventsData
    m_numControllerEvents::Cint
    m_controllerEvents::Ptr{b3VRControllerEvent}
end

struct b3KeyboardEvent
    m_keyCode::Cint
    m_keyState::Cint
end

struct b3KeyboardEventsData
    m_numKeyboardEvents::Cint
    m_keyboardEvents::Ptr{b3KeyboardEvent}
end

@cenum(eMouseEventTypeEnums,
    MOUSE_MOVE_EVENT = 1,
    MOUSE_BUTTON_EVENT = 2,
)

struct b3MouseEvent
    m_eventType::Cint
    m_mousePosX::Cfloat
    m_mousePosY::Cfloat
    m_buttonIndex::Cint
    m_buttonState::Cint
end

struct b3MouseEventsData
    m_numMouseEvents::Cint
    m_mouseEvents::Ptr{b3MouseEvent}
end

@cenum(b3NotificationType,
    SIMULATION_RESET = 0,
    BODY_ADDED = 1,
    BODY_REMOVED = 2,
    USER_DATA_ADDED = 3,
    USER_DATA_REMOVED = 4,
    LINK_DYNAMICS_CHANGED = 5,
    VISUAL_SHAPE_CHANGED = 6,
    TRANSFORM_CHANGED = 7,
    SIMULATION_STEPPED = 8,
)

struct b3BodyNotificationArgs
    m_bodyUniqueId::Cint
end

struct b3UserDataNotificationArgs
    m_bodyUniqueId::Cint
    m_linkIndex::Cint
    m_visualShapeIndex::Cint
    m_userDataId::Cint
    m_key::NTuple{256, UInt8}
end

struct b3LinkNotificationArgs
    m_bodyUniqueId::Cint
    m_linkIndex::Cint
end

struct b3VisualShapeNotificationArgs
    m_bodyUniqueId::Cint
    m_linkIndex::Cint
    m_visualShapeIndex::Cint
end

struct b3TransformChangeNotificationArgs
    m_bodyUniqueId::Cint
    m_linkIndex::Cint
    m_worldPosition::NTuple{3, Cdouble}
    m_worldRotation::NTuple{4, Cdouble}
    m_localScaling::NTuple{3, Cdouble}
end

struct b3Notification
    m_notificationType::Cint
end

struct b3ContactPointData
    m_contactFlags::Cint
    m_bodyUniqueIdA::Cint
    m_bodyUniqueIdB::Cint
    m_linkIndexA::Cint
    m_linkIndexB::Cint
    m_positionOnAInWS::NTuple{3, Cdouble}
    m_positionOnBInWS::NTuple{3, Cdouble}
    m_contactNormalOnBInWS::NTuple{3, Cdouble}
    m_contactDistance::Cdouble
    m_normalForce::Cdouble
    m_linearFrictionForce1::Cdouble
    m_linearFrictionForce2::Cdouble
    m_linearFrictionDirection1::NTuple{3, Cdouble}
    m_linearFrictionDirection2::NTuple{3, Cdouble}
end

@cenum(b3StateLoggingType,
    STATE_LOGGING_MINITAUR = 0,
    STATE_LOGGING_GENERIC_ROBOT = 1,
    STATE_LOGGING_VR_CONTROLLERS = 2,
    STATE_LOGGING_VIDEO_MP4 = 3,
    STATE_LOGGING_COMMANDS = 4,
    STATE_LOGGING_CONTACT_POINTS = 5,
    STATE_LOGGING_PROFILE_TIMINGS = 6,
    STATE_LOGGING_ALL_COMMANDS = 7,
    STATE_REPLAY_ALL_COMMANDS = 8,
    STATE_LOGGING_CUSTOM_TIMER = 9,
)

struct b3ContactInformation
    m_numContactPoints::Cint
    m_contactPointData::Ptr{b3ContactPointData}
end

struct b3RayData
    m_rayFromPosition::NTuple{3, Cdouble}
    m_rayToPosition::NTuple{3, Cdouble}
end

struct b3RayHitInfo
    m_hitFraction::Cdouble
    m_hitObjectUniqueId::Cint
    m_hitObjectLinkIndex::Cint
    m_hitPositionWorld::NTuple{3, Cdouble}
    m_hitNormalWorld::NTuple{3, Cdouble}
end

struct b3RaycastInformation
    m_numRayHits::Cint
    m_rayHits::Ptr{b3RayHitInfo}
end

struct RAY_DATA_UNION
    b::b3RayHitInfo
end

@cenum(b3VisualShapeDataFlags,
    eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = 1,
)

struct b3VisualShapeData
    m_objectUniqueId::Cint
    m_linkIndex::Cint
    m_visualGeometryType::Cint
    m_dimensions::NTuple{3, Cdouble}
    m_meshAssetFileName::NTuple{1024, UInt8}
    m_localVisualFrame::NTuple{7, Cdouble}
    m_rgbaColor::NTuple{4, Cdouble}
    m_tinyRendererTextureId::Cint
    m_textureUniqueId::Cint
    m_openglTextureId::Cint
end

struct b3VisualShapeInformation
    m_numVisualShapes::Cint
    m_visualShapeData::Ptr{b3VisualShapeData}
end

struct b3CollisionShapeData
    m_objectUniqueId::Cint
    m_linkIndex::Cint
    m_collisionGeometryType::Cint
    m_dimensions::NTuple{3, Cdouble}
    m_localCollisionFrame::NTuple{7, Cdouble}
    m_meshAssetFileName::NTuple{1024, UInt8}
end

struct b3CollisionShapeInformation
    m_numCollisionShapes::Cint
    m_collisionShapeData::Ptr{b3CollisionShapeData}
end

@cenum(eLinkStateFlags,
    ACTUAL_STATE_COMPUTE_LINKVELOCITY = 1,
    ACTUAL_STATE_COMPUTE_FORWARD_KINEMATICS = 2,
)

struct b3LinkState
    m_worldPosition::NTuple{3, Cdouble}
    m_worldOrientation::NTuple{4, Cdouble}
    m_localInertialPosition::NTuple{3, Cdouble}
    m_localInertialOrientation::NTuple{4, Cdouble}
    m_worldLinkFramePosition::NTuple{3, Cdouble}
    m_worldLinkFrameOrientation::NTuple{4, Cdouble}
    m_worldLinearVelocity::NTuple{3, Cdouble}
    m_worldAngularVelocity::NTuple{3, Cdouble}
    m_worldAABBMin::NTuple{3, Cdouble}
    m_worldAABBMax::NTuple{3, Cdouble}
end

@cenum(EnumExternalForceFlags,
    EF_LINK_FRAME = 1,
    EF_WORLD_FRAME = 2,
)
@cenum(EnumRenderer,
    ER_TINY_RENDERER = 65536,
    ER_BULLET_HARDWARE_OPENGL = 131072,
)
@cenum(EnumRendererAuxFlags,
    ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX = 1,
    ER_USE_PROJECTIVE_TEXTURE = 2,
    ER_NO_SEGMENTATION_MASK = 4,
)
@cenum(EnumCalculateInverseKinematicsFlags,
    IK_DLS = 0,
    IK_SDLS = 1,
    IK_HAS_TARGET_POSITION = 16,
    IK_HAS_TARGET_ORIENTATION = 32,
    IK_HAS_NULL_SPACE_VELOCITY = 64,
    IK_HAS_JOINT_DAMPING = 128,
    IK_HAS_CURRENT_JOINT_POSITIONS = 256,
    IK_HAS_MAX_ITERATIONS = 512,
    IK_HAS_RESIDUAL_THRESHOLD = 1024,
)
@cenum(b3ConfigureDebugVisualizerEnum,
    COV_ENABLE_GUI = 1,
    COV_ENABLE_SHADOWS = 2,
    COV_ENABLE_WIREFRAME = 3,
    COV_ENABLE_VR_TELEPORTING = 4,
    COV_ENABLE_VR_PICKING = 5,
    COV_ENABLE_VR_RENDER_CONTROLLERS = 6,
    COV_ENABLE_RENDERING = 7,
    COV_ENABLE_SYNC_RENDERING_INTERNAL = 8,
    COV_ENABLE_KEYBOARD_SHORTCUTS = 9,
    COV_ENABLE_MOUSE_PICKING = 10,
    COV_ENABLE_Y_AXIS_UP = 11,
    COV_ENABLE_TINY_RENDERER = 12,
    COV_ENABLE_RGB_BUFFER_PREVIEW = 13,
    COV_ENABLE_DEPTH_BUFFER_PREVIEW = 14,
    COV_ENABLE_SEGMENTATION_MARK_PREVIEW = 15,
    COV_ENABLE_PLANAR_REFLECTION = 16,
    COV_ENABLE_SINGLE_STEP_RENDERING = 17,
)
@cenum(b3AddUserDebugItemEnum,
    DEB_DEBUG_TEXT_ALWAYS_FACE_CAMERA = 1,
    DEB_DEBUG_TEXT_USE_TRUE_TYPE_FONTS = 2,
    DEB_DEBUG_TEXT_HAS_TRACKING_OBJECT = 4,
)
@cenum(eCONNECT_METHOD,
    eCONNECT_GUI = 1,
    eCONNECT_DIRECT = 2,
    eCONNECT_SHARED_MEMORY = 3,
    eCONNECT_UDP = 4,
    eCONNECT_TCP = 5,
    eCONNECT_EXISTING_EXAMPLE_BROWSER = 6,
    eCONNECT_GUI_SERVER = 7,
    eCONNECT_GUI_MAIN_THREAD = 8,
    eCONNECT_SHARED_MEMORY_SERVER = 9,
    eCONNECT_DART = 10,
    eCONNECT_MUJOCO = 11,
    eCONNECT_GRPC = 12,
    eCONNECT_PHYSX = 13,
)
@cenum(eURDF_Flags,
    URDF_USE_INERTIA_FROM_FILE = 2,
    URDF_USE_SELF_COLLISION = 8,
    URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,
    URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,
    URDF_RESERVED = 64,
    URDF_USE_IMPLICIT_CYLINDER = 128,
    URDF_GLOBAL_VELOCITIES_MB = 256,
    MJCF_COLORS_FROM_FILE = 512,
    URDF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,
    URDF_ENABLE_SLEEPING = 2048,
    URDF_INITIALIZE_SAT_FEATURES = 4096,
    URDF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,
    URDF_PARSE_SENSORS = 16384,
    URDF_USE_MATERIAL_COLORS_FROM_MTL = 32768,
    URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,
    URDF_MAINTAIN_LINK_ORDER = 131072,
    URDF_ENABLE_WAKEUP = 262144,
)
@cenum(eUrdfGeomTypes,
    GEOM_SPHERE = 2,
    GEOM_BOX = 3,
    GEOM_CYLINDER = 4,
    GEOM_MESH = 5,
    GEOM_PLANE = 6,
    GEOM_CAPSULE = 7,
    GEOM_UNKNOWN = 8,
)
@cenum(eUrdfCollisionFlags,
    GEOM_FORCE_CONCAVE_TRIMESH = 1,
    GEOM_CONCAVE_INTERNAL_EDGE = 2,
)
@cenum(eUrdfVisualFlags,
    GEOM_VISUAL_HAS_RGBA_COLOR = 1,
    GEOM_VISUAL_HAS_SPECULAR_COLOR = 2,
)
@cenum(eStateLoggingFlags,
    STATE_LOG_JOINT_MOTOR_TORQUES = 1,
    STATE_LOG_JOINT_USER_TORQUES = 2,
    STATE_LOG_JOINT_TORQUES = 3,
)
@cenum(eJointFeedbackModes,
    JOINT_FEEDBACK_IN_WORLD_SPACE = 1,
    JOINT_FEEDBACK_IN_JOINT_FRAME = 2,
)

struct b3PluginArguments
    m_text::NTuple{1024, UInt8}
    m_numInts::Cint
    m_ints::NTuple{128, Cint}
    m_numFloats::Cint
    m_floats::NTuple{128, Cdouble}
end

struct b3PhysicsSimulationParameters
    m_deltaTime::Cdouble
    m_simulationTimestamp::Cdouble
    m_gravityAcceleration::NTuple{3, Cdouble}
    m_numSimulationSubSteps::Cint
    m_numSolverIterations::Cint
    m_useRealTimeSimulation::Cint
    m_useSplitImpulse::Cint
    m_splitImpulsePenetrationThreshold::Cdouble
    m_contactBreakingThreshold::Cdouble
    m_internalSimFlags::Cint
    m_defaultContactERP::Cdouble
    m_collisionFilterMode::Cint
    m_enableFileCaching::Cint
    m_restitutionVelocityThreshold::Cdouble
    m_defaultNonContactERP::Cdouble
    m_frictionERP::Cdouble
    m_defaultGlobalCFM::Cdouble
    m_frictionCFM::Cdouble
    m_enableConeFriction::Cint
    m_deterministicOverlappingPairs::Cint
    m_allowedCcdPenetration::Cdouble
    m_jointFeedbackMode::Cint
    m_solverResidualThreshold::Cdouble
    m_contactSlop::Cdouble
    m_enableSAT::Cint
    m_constraintSolverType::Cint
    m_minimumSolverIslandSize::Cint
    m_reportSolverAnalytics::Cint
end

@cenum(eConstraintSolverTypes,
    eConstraintSolverLCP_SI = 1,
    eConstraintSolverLCP_PGS = 2,
    eConstraintSolverLCP_DANTZIG = 3,
    eConstraintSolverLCP_LEMKE = 4,
    eConstraintSolverLCP_NNCG = 5,
    eConstraintSolverLCP_BLOCK_PGS = 6,
)

struct b3ForwardDynamicsAnalyticsIslandData
    m_islandId::Cint
    m_numBodies::Cint
    m_numContactManifolds::Cint
    m_numIterationsUsed::Cint
    m_remainingLeastSquaresResidual::Cdouble
end

struct b3ForwardDynamicsAnalyticsArgs
    m_numSteps::Cint
    m_numIslands::Cint
    m_numSolverCalls::Cint
    m_islandData::NTuple{1024, b3ForwardDynamicsAnalyticsIslandData}
end

@cenum(eFileIOActions,
    eAddFileIOAction = 1024,
    eRemoveFileIOAction = 1025,
)
@cenum(eFileIOTypes,
    ePosixFileIO = 1,
    eZipFileIO = 2,
    eCNSFileIO = 3,
    eInMemoryFileIO = 4,
)
