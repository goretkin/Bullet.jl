using Bullet.BulletCxx: get_collider, get_collision_flags

import .Bullet.Julian.CoordinateTransformations: Translation, Rotations, LinearMap, compose
TX(x,y,z,r,p,yaw) = compose(Translation(x,y,z), LinearMap(Rotations.RotXYZ(r, p, yaw)))

sm = Bullet.connect()

floor_id = Bullet.load_urdf(sm, Bullet.data_planeMesh_urdf)

@testset "collision flags" begin
  collision_group_all = reinterpret(Cint, 0xffffffff)
  cube_ids = [Bullet.load_urdf(sm, Bullet.data_cube_small_urdf) for _ = 1:5]
  for i in 1:5
    cube_id = cube_ids[i]
    Bullet.set_base_pose(sm, cube_id, TX(0,0,i*0.1,0,0,0))

    @show get_collision_flags(get_collider(sm, cube_id, Bullet.LINK_ID_NONE))
    collision_filter_mask = reinterpret(Cint, 0x00000001 << i)
    Bullet.set_collision_flags(sm, cube_id, Bullet.LINK_ID_NONE, collision_group_all, collision_filter_mask)
  end

  for i in 1:5
    collision_flags = get_collision_flags(get_collider(sm, cube_ids[i], Bullet.LINK_ID_NONE))
    @test collision_flags.group == collision_group_all
    @test collision_flags.mask == (0x1 << i)
  end
end

Bullet.disconnect()
