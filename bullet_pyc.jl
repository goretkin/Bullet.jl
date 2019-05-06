bullet_repo = "/Users/goretkin/repos/bullet3/"
bullet_so = joinpath(bullet_repo, "build/examples/c_api/bullet_c_api.dylib")

const libclang = bullet_so
using CEnum
include("ctypes.jl")
include("libclang_common.jl")
include("libclang_api.jl")


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

#sm = b3ConnectPhysicsDirect()
sm = b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(0, [])

@show submit_client_command_and_wait_status(sm, b3InitResetSimulationCommand(sm))

urdfpath = joinpath(bullet_repo, "data/planeMesh.urdf")
load_urdf(sm, urdfpath)

for i in 1:5
  cube = load_urdf(sm, joinpath(bullet_repo, "data", "cube_small.urdf"),
    position=[0, i*0.02, (i+1)*0.2])
  @show cube
end

set_gravity(sm, [0, 0, -9.8])

# b3DisconnectSharedMemory(sm);


for _ = 1:600
  step_simulation(sm)
  sleep(1/60)
end