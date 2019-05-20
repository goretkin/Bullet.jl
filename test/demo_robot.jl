import AtlasRobot

sm = Bullet.connect()
@testset "atlas robot" begin
robot_id = Bullet.load_urdf(sm, AtlasRobot.urdfpath())

all_joints = Bullet.get_all_joints(sm, robot_id)
atlas_revolute_dofs = 30
@test atlas_revolute_dofs == length(filter(isequal(Bullet.Raw.eRevoluteType), all_joints.joint_types))
end

Bullet.disconnect()
