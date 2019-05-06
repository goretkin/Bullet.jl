using StaticArrays
using EponymTuples
using OffsetArrays

include("../src/load.jl")
include("../src/wrap.jl")

bullet_repo = "/Users/goretkin/repos/bullet3/"

#sm = b3ConnectPhysicsDirect()
sm = b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(0, [])

@show submit_client_command_and_wait_status(sm, b3InitResetSimulationCommand(sm))

urdfpath = joinpath(bullet_repo, "data/planeMesh.urdf")
floor_id = load_urdf(sm, urdfpath)

urdfpath = "/Users/goretkin/projects/VAMP.jl/data/movo.urdf"
movo_id = load_urdf(sm, urdfpath)


for i in 1:5
  cube = load_urdf(sm, joinpath(bullet_repo, "data", "cube_small.urdf"),
    position=[0, i*0.02, (i+1)*0.2])
  @show cube
end

set_gravity(sm, [0, 0, -9.8])

# b3DisconnectSharedMemory(sm);

timer = Timer((_ -> step_simulation(sm)), 0.0, interval=1/60)