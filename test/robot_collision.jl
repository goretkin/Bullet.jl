import AtlasRobot

import .Bullet.Julian.CoordinateTransformations: Translation, Rotations, LinearMap, compose
import .Bullet.Julian: ColorTypes
TX(x,y,z,r,p,yaw) = compose(Translation(x,y,z), LinearMap(Rotations.RotXYZ(r, p, yaw)))

sm = Bullet.connect(
  #;kind=:gui
)


box_id = Bullet.create_body_box(sm, [1.0, 0.1, 1.0], TX(0.5,0.5,1.0,0,0,0), ColorTypes.RGBA(1,0,0,0.1))

robot_id = Bullet.load_urdf(sm, AtlasRobot.urdfpath())
robot_bm = Bullet.BodyManager(sm, robot_id)

cpi = Bullet.get_contact_point_information(sm, robot_id, box_id, 0.001)


contacts = [unsafe_load(cpi.m_contactPointData, i) for i=1:cpi.m_numContactPoints]

for contact in contacts
  link_id = contact.m_linkIndexA
  if link_id == -1; continue end
  Bullet.set_color(sm, ColorTypes.RGBA(0.5, 0.5, 0.5, 0.5); body_id=robot_id, link_id=link_id)
end

@testset "collision" begin
  @test Set([c.m_linkIndexA for c in contacts]) == Set([4,5])
end

Bullet.disconnect()