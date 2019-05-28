connection_state = nothing
timer_state = nothing

function submit_client_command_and_wait_status_checked(sm, command; checked_status)
  status_handle = Raw.b3SubmitClientCommandAndWaitStatus(sm, command)
  status = Raw.EnumSharedMemoryServerStatus(Raw.b3GetStatusType(status_handle))
  if checked_status != status
    error("expected $(checked_status), got $(status)")
  end
  return status_handle
end

function reset_simulation(sm)
  submit_client_command_and_wait_status_checked(
    sm,
    Raw.b3InitResetSimulationCommand(sm)
    ;
    checked_status=Raw.CMD_RESET_SIMULATION_COMPLETED)
end

function handle_gui(sm)
  # Just a no-op command.
  submit_client_command_and_wait_status_checked(
    sm,
    Raw.b3InitRequestPhysicsParamCommand(sm)
    ;
    checked_status= Raw.CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED
  )
end

function setup_gui_timer()
  global connection_state
  global timer_state
  if connection_state == nothing
    error("not connected")
  end

  if timer_state  == nothing
    timer_state = Timer((_ -> handle_gui(connection_state.handle)), 0.0, interval=1/30)
  end
end

function connect(;kind=:direct, do_reset_sim=true, )
  global connection_state

  if connection_state == nothing
    if kind == :direct
      handle = Raw.b3ConnectPhysicsDirect()
      connection_state = (kind=kind, handle=handle)
    elseif kind == :gui
      handle = Raw.b3CreateInProcessPhysicsServerAndConnectMainThread(0, [])
      connection_state = (kind=kind, handle=handle)
    end

    if connection_state == nothing
      error("unknown kind: $(kind)")
    end
  end

  if connection_state.kind == kind
    if do_reset_sim
      reset_simulation(connection_state.handle)
    end
    return connection_state.handle
  else
    error("Can't switch kind after connection, unless you like segfaults")
  end
end

function disconnect()
  global connection_state
  if connection_state == nothing
    error("not connected")
  end

  if connection_state.kind == :direct
    Raw.b3DisconnectSharedMemory(connection_state.handle)
  else
    error("refuse to disconnect from GUI")
  end
  connection_state = nothing
end

function load_urdf(sm, urdfpath; position=[0, 0, 0], orientation=[0, 0, 0, 1], flags=0)
  command = Raw.b3LoadUrdfCommandInit(sm, urdfpath)
  Raw.b3LoadUrdfCommandSetFlags(command, flags)

  Raw.b3LoadUrdfCommandSetStartPosition(command, position...)
  Raw.b3LoadUrdfCommandSetStartOrientation(command, orientation...)

  status_handle = submit_client_command_and_wait_status_checked(sm, command; checked_status=Raw.CMD_URDF_LOADING_COMPLETED)
  bodyUniqueId = Raw.b3GetStatusBodyIndex(status_handle);
  return bodyUniqueId
end


function step_simulation(sm)
  status_handle = Raw.b3SubmitClientCommandAndWaitStatus(sm, Raw.b3InitStepSimulationCommand(sm))
  status = Raw.b3GetStatusType(status_handle)
end


function set_gravity(sm, gravity)
  command = Raw.b3InitPhysicsParamCommand(sm)

  Safe.PhysicsParamSetGravity(command, gravity)

  submit_client_command_and_wait_status_checked(sm, command; checked_status=Raw.CMD_CLIENT_COMMAND_COMPLETED)
end


function get_joint_info(sm, body_id, joint_id)
  ji = Ref{Raw.b3JointInfo}()
  @assert 1 == Raw.b3GetJointInfo(sm, body_id, joint_id, ji)
  return ji[]
end


function c_string(data::NTuple{N, UInt8}) where {N}
  data = (data[1:(findfirst(isequal(0), data))-1])
  return String(SArray{Tuple{length(data)}}(data))
end


function get_all_joints(sm, body_id)
  num_joints = Raw.b3GetNumJoints(sm, body_id)
  joint_names = OffsetArray{String}(undef, 0:num_joints-1)
  link_names = OffsetArray{String}(undef, 0:num_joints-1)
  joint_types = OffsetArray{Raw.JointType}(undef, 0:num_joints-1)

  joint_info_ref = Ref{Raw.b3JointInfo}()
  for joint_id = 0:num_joints-1
    @assert 1 == Raw.b3GetJointInfo(sm, body_id, joint_id, joint_info_ref)
    joint_names[joint_id] = c_string(joint_info_ref[].m_jointName)
    link_names[joint_id] = c_string(joint_info_ref[].m_linkName)
    joint_types[joint_id] = Raw.JointType(joint_info_ref[].m_jointType)
  end
  return @eponymtuple(joint_names, link_names, joint_types)
end


function get_sensor_state(sm, body_id, joint_id)
  command_handle = Raw.b3RequestActualStateCommandInit(sm, body_id)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
  sensor_state_ref = Ref{Raw.b3JointSensorState}()
  @assert 1 == Raw.b3GetJointState(sm, status_handle, joint_id, sensor_state_ref)
  return sensor_state_ref[]
end


function get_joint_position(sm, body_id, joint_id)
  get_sensor_state(sm, body_id, joint_id).m_jointPosition
end


function set_joint_position(sm, body_id, joint_id; position=nothing, velocity=nothing)
  command_handle = Raw.b3CreatePoseCommandInit(sm, body_id)

  position != nothing && Raw.b3CreatePoseCommandSetJointPosition(sm, command_handle, joint_id, position);
  velocity != nothing && Raw.b3CreatePoseCommandSetJointVelocity(sm, command_handle, joint_id, velocity);

  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CLIENT_COMMAND_COMPLETED)
end


function get_base_pose(sm, body_id)
  pointer_ref = Ref{Ptr{Cdouble}}()

  command_handle = Raw.b3RequestActualStateCommandInit(sm, body_id)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_ACTUAL_STATE_UPDATE_COMPLETED)

  null = Ptr{Any}(0)
  Raw.b3GetStatusActualState(
    status_handle, null #= body_unique_id =#,
    null #= num_degree_of_freedom_q =#, null #= num_degree_of_freedom_u =#,
    null #=root_local_inertial_frame=#, pointer_ref,
    null #= actual_state_q_dot =#, null #= joint_reaction_forces =#)

  pos_quat = unsafe_wrap(Array, pointer_ref[], (7,), own=false)
  pos = SVector(pos_quat[1:3]...)

  quat = let (x, y, z, w) = pos_quat[4:7]
    CoordinateTransformations.Rotations.Quat(w, x, y, z, false)
  end

  pose = compose(Translation(pos), LinearMap(quat))
  return pose
end


function set_base_pose(sm, body_id, transformation)
  command_handle = Raw.b3CreatePoseCommandInit(sm, body_id)

  translation = transformation.translation
  rotation = transformation.linear

  Safe.CreatePoseCommandSetBasePosition(command_handle, translation)
  Safe.CreatePoseCommandSetBaseOrientation(command_handle, rotation)

  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CLIENT_COMMAND_COMPLETED)
end


function get_links_poses(sm, body_id)
  command_handle = Raw.b3RequestActualStateCommandInit(sm, body_id)
  Raw.b3RequestActualStateCommandComputeForwardKinematics(command_handle, true)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_ACTUAL_STATE_UPDATE_COMPLETED)

  link_index = 1
  link_state_ref = Ref{Raw.b3LinkState}()
  Raw.b3GetLinkState(sm, status_handle, link_index, link_state_ref)
  return link_state_ref[]
end


struct BodyManager
  sm::Ptr{Raw.b3PhysicsClientHandle__}
  body_id::Int
  joint_names::OffsetArray{Symbol, 1, Vector{Symbol}}
  link_names::OffsetArray{Symbol, 1, Vector{Symbol}}
  joint_types::OffsetArray{Raw.JointType, 1, Vector{Raw.JointType}}
  joint::Dict{Symbol, Int}
  link::Dict{Symbol, Int}
end


function BodyManager(sm, body_id::Integer)
  all_joints = Bullet.get_all_joints(sm, body_id)
  BodyManager(sm, body_id,
    map(Symbol, all_joints.joint_names),
    map(Symbol, all_joints.link_names),
    all_joints.joint_types,
    Dict(Symbol(all_joints.joint_names[i])=>i for i = eachindex(all_joints.joint_names)),
    Dict(Symbol(all_joints.link_names[i])=>i for i = eachindex(all_joints.link_names))
  )
end



function get_contact_point_information(sm, body_a_id, link_a_id, body_b_id, link_b_id, distance)
  command_handle = Raw.b3InitClosestDistanceQuery(sm)
  Raw.b3SetClosestDistanceThreshold(command_handle, distance)
  Raw.b3SetClosestDistanceFilterBodyA(command_handle, body_a_id)
  Raw.b3SetClosestDistanceFilterBodyB(command_handle, body_b_id)

  Raw.b3SetClosestDistanceFilterLinkA(command_handle, link_a_id)
  Raw.b3SetClosestDistanceFilterLinkB(command_handle, link_b_id)

  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CONTACT_POINT_INFORMATION_COMPLETED)

  contact_point_data = Ref{Raw.b3ContactInformation}()
  Raw.b3GetContactPointInformation(sm, contact_point_data)
  return contact_point_data[]
end


function get_contact_point_information(sm, body_a_id, body_b_id, distance)
  command_handle = Raw.b3InitClosestDistanceQuery(sm)
  Raw.b3SetClosestDistanceThreshold(command_handle, distance)
  Raw.b3SetClosestDistanceFilterBodyA(command_handle, body_a_id)
  Raw.b3SetClosestDistanceFilterBodyB(command_handle, body_b_id)

  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CONTACT_POINT_INFORMATION_COMPLETED)

  contact_point_data = Ref{Raw.b3ContactInformation}()
  Raw.b3GetContactPointInformation(sm, contact_point_data)
  return contact_point_data[]
end

function set_debug_object_color(sm, body_id, link_id, color_rgb)
  #=
    only visible in wireframe mode of Bullet debug visualizer (hit the W key)
  =#
  command_handle = Raw.b3InitDebugDrawingCommand(sm)
  Raw.b3SetDebugObjectColor(command_handle, body_id, link_id, color_rgb);
  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_USER_DEBUG_DRAW_COMPLETED);
end

function set_collision_filter_pair(sm, body_a_id, link_a_id, body_b_id, link_b_id, enable_collision)
  command_handle = Raw.b3CollisionFilterCommandInit(sm)
	Raw.b3SetCollisionFilterPair(command_handle, body_a_id, body_b_id, link_a_id, link_b_id, enable_collision);
  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CLIENT_COMMAND_COMPLETED);
end


function create_body_box(sm, half_extents, transformation, color)
  half_extents = convert(Vector{Float64}, half_extents)
  @assert length(half_extents) == 3

  command_handle = Raw.b3CreateCollisionShapeCommandInit(sm)
  # id within a possibly-compound collision shape
  collision_shape_id = Raw.b3CreateCollisionShapeAddBox(command_handle, half_extents)
  @assert collision_shape_id != -1

  Safe.CreateCollisionShapeSetChildTransform(command_handle, collision_shape_id, [0, 0, 0], one(Julian.Quat))

  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CREATE_COLLISION_SHAPE_COMPLETED)
  collision_shape_unique_id = Raw.b3GetStatusCollisionShapeUniqueId(status_handle)
  @assert collision_shape_unique_id != -1

  #------------------
  command_handle = Raw.b3CreateVisualShapeCommandInit(sm)
  visual_shape_id = Raw.b3CreateVisualShapeAddBox(command_handle, half_extents)
  @assert visual_shape_id != -1

  Safe.CreateVisualShapeSetChildTransform(command_handle, visual_shape_id, [0, 0, 0], one(Julian.Quat))
  Safe.CreateVisualShapeSetRGBAColor(command_handle, visual_shape_id, color)
  Safe.CreateVisualShapeSetSpecularColor(command_handle, visual_shape_id, Julian.ColorTypes.RGB(1.0, 1.0, 1.0))

  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CREATE_VISUAL_SHAPE_COMPLETED)
  visual_shape_unique_id = Raw.b3GetStatusVisualShapeUniqueId(status_handle)
  @assert visual_shape_unique_id != -1

  #-----------

  translation = transformation.translation
  rotation = transformation.linear

  command_handle = Raw.b3CreateMultiBodyCommandInit(sm)
  baseMass = 0
  baseInertialFramePosition = [0,0,0]
  baseInertialFrameOrientation = one(Julian.Quat)

  Safe.CreateMultiBodyBase(command_handle,
    baseMass, collision_shape_unique_id, visual_shape_unique_id, translation, rotation, baseInertialFramePosition, baseInertialFrameOrientation);

  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_CREATE_MULTI_BODY_COMPLETED)
  body_id = Raw.b3GetStatusBodyIndex(status_handle)
  return body_id
end

function set_color(sm, color; body_id, link_id=-1, shape_id=-1)
  command_handle = Raw.b3InitUpdateVisualShape2(sm, body_id, link_id, shape_id);
  Safe.UpdateVisualShapeRGBAColor(command_handle, color)
  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Bullet.Raw.CMD_VISUAL_SHAPE_UPDATE_COMPLETED)
end

function raycast(sm, ray_from, ray_to)
  command_handle = Safe.CreateRaycastCommandInit(sm, ray_from, ray_to)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
  rci_ref = Ref{Raw.b3RaycastInformation}()
  Raw.b3GetRaycastInformation(sm, rci_ref)
  rci = rci_ref[]

  @assert rci.m_numRayHits == 1
  ray_hit = unsafe_load(rci.m_rayHits)
  return ray_hit
end

function raycast_batch(sm, rays_from::Array{Cdouble, 2}, rays_to::Array{Cdouble, 2},)
  # must ensure memory layout matches Bullet's
  @assert size(rays_from, 1) == 3
  @assert size(rays_to, 1) == 3
  @assert size(rays_from, 2) == size(rays_to, 2)
  N = size(rays_from, 2)
  @assert N <= Raw.MAX_RAY_INTERSECTION_BATCH_SIZE

  command_handle = Raw.b3CreateRaycastBatchCommandInit(sm)
  #Raw.b3RaycastBatchSetNumThreads(command_handle, 1)

  Raw.b3RaycastBatchAddRays(sm, command_handle, rays_from, rays_to, N)

  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)

  rci_ref = Ref{Raw.b3RaycastInformation}()
  Raw.b3GetRaycastInformation(sm, rci_ref)

  rci = rci_ref[]
  @assert rci.m_numRayHits == N
  rci
end

function user_debug_draw_add_line(sm, from, to, color, line_width, life_time)
  command_handle = Safe.InitUserDebugDrawAddLine3D(sm, from, to, color, line_width, life_time)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=Raw.CMD_USER_DEBUG_DRAW_COMPLETED)
  debugItemUniqueId = Raw.b3GetDebugItemUniqueId(status_handle)
  return debugItemUniqueId
end
