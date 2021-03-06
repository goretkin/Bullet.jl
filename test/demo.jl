sm = Bullet.connect()

floor_id = Bullet.load_urdf(sm, Bullet.data_planeMesh_urdf)

cube_ids = []
@testset "load_urdf and get_base_pose" begin
  for i in 1:5
    starting_position = Float32[0, i*0.2, (i+1)*0.2]
    cube_id = Bullet.load_urdf(sm, Bullet.data_cube_small_urdf,
      position=starting_position)
    push!(cube_ids, cube_id)
    # with default Bullet compile, Float32 roundtrips
    @test Bullet.get_base_pose(sm, cube_ids[i]).translation == starting_position
  end
end

Bullet.set_gravity(sm, [0, 0, -9.8])

traj = typeof(Bullet.get_base_pose(sm, cube_ids[1]))[]

sim_steps = nothing
for i = 1:500
  global sim_steps = i
  push!(traj, Bullet.get_base_pose(sm, cube_ids[1]))
  Bullet.step_simulation(sm)
  # TODO check changes in rotation too.
  if i > 1 && isapprox(traj[end].translation, traj[end-1].translation)
    break
  end
end

@test 265 < sim_steps < 275

z_pos = getindex.(getfield.(traj, Ref(:translation)), Ref(3))
z_vel = diff(z_pos)
z_acc = diff(z_vel)
z_jerk = diff(z_acc)  # physically, jerk = 0 in freefall

@test all(z_jerk[1:50]  .< (50 * eps(Float32)))


begin
  using Bullet.CoordinateTransformations: Translation, Rotations, LinearMap, compose
  using .Rotations: rotation_angle

  set_transformation = compose(Translation(1,2,3), LinearMap(Rotations.RotXYZ(0.1, 0.1, 0.1)))

  Bullet.set_base_pose(sm, cube_ids[1], set_transformation)
  test_transformation = Bullet.get_base_pose(sm, cube_ids[1])
  @test rotation_angle(set_transformation.linear \ test_transformation.linear) < 10eps(Float32)
  @test isapprox(set_transformation.translation, test_transformation.translation)
end

Bullet.disconnect()
