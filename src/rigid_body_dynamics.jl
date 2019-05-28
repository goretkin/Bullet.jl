module RigidBodyDynamicsAdapter

import ..Bullet

import RigidBodyDynamics
const rbd = RigidBodyDynamics

import CoordinateTransformations
const CT = CoordinateTransformations


const bu_rbd_joints = [
  (Bullet.Raw.eRevoluteType, rbd.Revolute),
  (Bullet.Raw.ePrismaticType, rbd.Prismatic),
  (Bullet.Raw.eFixedType, rbd.Fixed),
]

const bu_to_rbd_joint_type_map = Dict(bu=>rbd for (bu, rbd) in bu_rbd_joints)

# TODO make an Adapter type to hoist out all string comparisons and dictionaries
function update_state(sm, bullet_body_id, mechanism_state::rbd.MechanismState)
  all_joints_bullet = Bullet.get_all_joints(sm, bullet_body_id)
  all_joints_rbd = rbd.joints(mechanism_state.mechanism)
  rbd_joint_by_name = Dict(joint.name=>joint for joint in all_joints_rbd)

  for joint_name in all_joints_bullet.joint_names
    rbd_joint = rbd_joint_by_name[joint_name]
    q = rbd.configuration(mechanism_state, rbd_joint_by_name[joint_name])
    q̇ = rbd.velocity(mechanism_state, rbd_joint_by_name[joint_name])
    rbd.joint_type(rbd_joint)
    (bullet_joint_id, ) = findall(all_joints_bullet.joint_names .== Ref(joint_name))
    # TODO check joint type
    bullet_joint_type = all_joints_bullet.joint_types[bullet_joint_id]
    rbd_joint_type = rbd.joint_type(rbd_joint)
    @assert typeof(rbd_joint_type) <: bu_to_rbd_joint_type_map[bullet_joint_type]

    if bullet_joint_type == Bullet.Raw.eRevoluteType || bullet_joint_type == Bullet.Raw.ePrismaticType
      Bullet.set_joint_position(sm, bullet_body_id, bullet_joint_id; position=q[], velocity=q̇[])
    elseif bullet_joint_type == Bullet.Raw.eFixedType
      # no configuration
    else
      error("Unsupported joint type: $bullet_joint_type")
    end
  end

  # TODO don't hardcode this joint name.
  floating_joint = rbd.findjoint(mechanism_state.mechanism, "floating_joint")
  @assert typeof(rbd.joint_type(floating_joint)) <: rbd.QuaternionFloating
  # TODO safer way to access RBD quaternion and translation
  quat_tran = rbd.configuration(mechanism_state, floating_joint)
  quat_wxyz = quat_tran[1:4]
  rot = CT.LinearMap(CoordinateTransformations.Quat(quat_wxyz...))
  tran = CT.Translation(quat_tran[5:7]...)
  Bullet.set_base_pose(sm, bullet_body_id, CT.compose(tran, rot))
  # TODO set base twist
end
end