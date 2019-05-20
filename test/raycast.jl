import Bullet

import .Bullet.Julian.CoordinateTransformations: Translation, Rotations, LinearMap, compose
import .Bullet.Julian: ColorTypes
TX(x,y,z,r,p,yaw) = compose(Translation(x,y,z), LinearMap(Rotations.RotXYZ(r, p, yaw)))

sm = Bullet.connect(
  #;kind=:gui
)


box_id = Bullet.create_body_box(sm, [1.0, 0.1, 1.0], TX(0.5,0.5,1.0,0,0,0), ColorTypes.RGBA(1,0,0,0.1))

eye = [0, 0, 0.5]
targets = [[1.0, y, z] for y = -2:0.3:2, z = -2:0.3:2]

single_time = @elapsed (hits = [Bullet.raycast(sm, eye, target) for target in targets])
@show single_time

if Bullet.connection_state.kind == :gui
  for raycast_i in eachindex(targets)
    did_hit = hits[raycast_i].m_hitObjectUniqueId != -1
    #did_hit = false
    color = if did_hit ColorTypes.RGBA(1.0, 1.0, 0.0, 0.5) else ColorTypes.RGBA(1.0, 1.0, 1.0, 0.5) end
    to = if did_hit collect(hits[raycast_i].m_hitPositionWorld) else targets[raycast_i] end

    Bullet.user_debug_draw_add_line(sm, eye, to, color, 1.0, 20.0)
  end
end

hit_image = [hit.m_hitObjectUniqueId for hit in hits]

@testset "hit image" begin
  @test all(hit_image[1:8, :] .== -1)
  @test all(hit_image[10:end, 7:end] .== 0)
end

@testset "batch raycast" begin
  targets_flat = hcat(targets[:]...)
  sources_flat = hcat((eye for _ in 1:size(targets_flat, 2))...)
  batch_time_first = @elapsed (raycast_info = Bullet.raycast_batch(sm, sources_flat, targets_flat))
  @show batch_time_first # includes compilation time.
  batch_time = @elapsed (raycast_info = Bullet.raycast_batch(sm, sources_flat, targets_flat))
  @show batch_time

  lin_idxs = LinearIndices(targets)
  hits_batch = [unsafe_load(raycast_info.m_rayHits, lin_idxs[cart_idx]) for cart_idx in CartesianIndices(targets)]

  hit_batch_image = [hit.m_hitObjectUniqueId for hit in hits_batch]

  @test all(hit_batch_image .== hit_image)
end

Bullet.disconnect()
