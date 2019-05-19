import AtlasRobot

sm = Bullet.Raw.b3ConnectPhysicsDirect()

Bullet.submit_client_command_and_wait_status_checked(sm, Bullet.Raw.b3InitResetSimulationCommand(sm); checked_status=Bullet.Raw.CMD_RESET_SIMULATION_COMPLETED)

@testset "atlas robot" begin
robot_id = Bullet.load_urdf(sm, AtlasRobot.urdfpath())

all_joints = Bullet.get_all_joints(sm, robot_id)
atlas_revolute_dofs = 30
@test atlas_revolute_dofs == length(filter(isequal(Bullet.Raw.eRevoluteType), all_joints.joint_types))
end

Bullet.Raw.b3DisconnectSharedMemory(sm);
