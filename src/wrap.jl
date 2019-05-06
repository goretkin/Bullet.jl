function submit_client_command_and_wait_status(sm, command)
  EnumSharedMemoryServerStatus(b3GetStatusType(b3SubmitClientCommandAndWaitStatus(sm, command)))
end


function load_urdf(sm, urdfpath; position=[0, 0, 0], orientation=[0, 0, 0, 1], flags=0)
  command = b3LoadUrdfCommandInit(sm, urdfpath)
  b3LoadUrdfCommandSetFlags(command, flags)

  b3LoadUrdfCommandSetStartPosition(command, position...)
  b3LoadUrdfCommandSetStartOrientation(command, orientation...)

  status_handle = b3SubmitClientCommandAndWaitStatus(sm, command)
  status = EnumSharedMemoryServerStatus(b3GetStatusType(status_handle))
  if status != CMD_URDF_LOADING_COMPLETED; error(status); end
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

  status_handle = b3SubmitClientCommandAndWaitStatus(sm, command)
  @assert(b3GetStatusType(status_handle) == CMD_CLIENT_COMMAND_COMPLETED)
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