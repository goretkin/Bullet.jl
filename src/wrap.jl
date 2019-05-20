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

function setup_timer()
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
      nothing
    elseif kind == :gui
      handle = Raw.b3CreateInProcessPhysicsServerAndConnectMainThread(0, [])
      connection_state = (kind=kind, handle=handle)
      Bullet.setup_timer()
      nothing
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
