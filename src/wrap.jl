function submit_client_command_and_wait_status_checked(sm, command; checked_status)
  status_handle = b3SubmitClientCommandAndWaitStatus(sm, command)
  status = EnumSharedMemoryServerStatus(b3GetStatusType(status_handle))
  if checked_status != status
    error("expected $(checked_status), got $(status)")
  end
  return status_handle
end



function load_urdf(sm, urdfpath; position=[0, 0, 0], orientation=[0, 0, 0, 1], flags=0)
  command = b3LoadUrdfCommandInit(sm, urdfpath)
  b3LoadUrdfCommandSetFlags(command, flags)

  b3LoadUrdfCommandSetStartPosition(command, position...)
  b3LoadUrdfCommandSetStartOrientation(command, orientation...)

  status_handle = submit_client_command_and_wait_status_checked(sm, command; checked_status=CMD_URDF_LOADING_COMPLETED)
  bodyUniqueId = b3GetStatusBodyIndex(status_handle);
  return bodyUniqueId
end


function step_simulation(sm)
  status_handle = b3SubmitClientCommandAndWaitStatus(sm, b3InitStepSimulationCommand(sm))
  status = b3GetStatusType(status_handle)
end


function set_gravity(sm, gravity)
  command = b3InitPhysicsParamCommand(sm)

  b3PhysicsParamSetGravity(command, gravity...)

  submit_client_command_and_wait_status_checked(sm, command; checked_status=CMD_CLIENT_COMMAND_COMPLETED)
end


function get_joint_info(sm, body_id, joint_id)
  ji = Ref{b3JointInfo}()
  @assert 1 == b3GetJointInfo(sm, body_id, joint_id, ji)
  return ji[]
end


function c_string(data::NTuple{N, UInt8}) where {N}
  data = (data[1:(findfirst(isequal(0), data))-1])
  return String(SArray{Tuple{length(data)}}(data))
end


function get_all_joints(sm, body_id)
  num_joints = b3GetNumJoints(sm, body_id)
  joint_names = OffsetArray{String}(undef, 0:num_joints-1)
  link_names = OffsetArray{String}(undef, 0:num_joints-1)
  joint_types = OffsetArray{JointType}(undef, 0:num_joints-1)

  joint_info_ref = Ref{b3JointInfo}()
  for joint_id = 0:num_joints-1
    @assert 1 == b3GetJointInfo(sm, body_id, joint_id, joint_info_ref)
    joint_names[joint_id] = c_string(joint_info_ref[].m_jointName)
    link_names[joint_id] = c_string(joint_info_ref[].m_linkName)
    joint_types[joint_id] = JointType(joint_info_ref[].m_jointType)
  end
  return @eponymtuple(joint_names, link_names, joint_types)
end


function get_sensor_state(sm, body_id, joint_id)
  command_handle = b3RequestActualStateCommandInit(sm, body_id)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=CMD_ACTUAL_STATE_UPDATE_COMPLETED)
  sensor_state_ref = Ref{b3JointSensorState}()
  @assert 1 == b3GetJointState(sm, status_handle, joint_id, sensor_state_ref)
  return sensor_state_ref[]
end


function get_joint_position(sm, body_id, joint_id)
  get_sensor_state(sm, body_id, joint_id).m_jointPosition
end


function set_joint_position(sm, body_id, joint_id; position=nothing, velocity=nothing)
  command_handle = b3CreatePoseCommandInit(sm, body_id)

  position != nothing && b3CreatePoseCommandSetJointPosition(sm, command_handle, joint_id, position);
  velocity != nothing && b3CreatePoseCommandSetJointVelocity(sm, command_handle, joint_id, velocity);

  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=CMD_CLIENT_COMMAND_COMPLETED)

end


function get_base_pose(sm, body_id)
  pointer_ref = Ref{Ptr{Cdouble}}()

  command_handle = b3RequestActualStateCommandInit(sm, body_id)
  status_handle = submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=CMD_ACTUAL_STATE_UPDATE_COMPLETED)

  null = Ptr{Any}(0)
  b3GetStatusActualState(
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
  command_handle = b3CreatePoseCommandInit(sm, body_id)

  translation = transformation.translation
  rotation = transformation.linear

  let (x, y, z) = transformation.translation
    b3CreatePoseCommandSetBasePosition(command_handle, x, y, z)
  end

  let q = CoordinateTransformations.Rotations.Quat(rotation)
    b3CreatePoseCommandSetBaseOrientation(command_handle, q.x, q.y, q.z, q.w)
  end

  submit_client_command_and_wait_status_checked(sm, command_handle; checked_status=CMD_CLIENT_COMMAND_COMPLETED)
end

