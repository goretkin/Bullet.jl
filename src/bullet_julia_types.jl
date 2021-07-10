module Julian
export
  bullet_color_alpha,
    bullet_color,
    bullet_quaternion,
    bullet_vector3

import Rotations
using Rotations: Quat

import ColorTypes
using ColorTypes: red, green, blue, alpha

bullet_color_alpha(T, color) = T[red(color), green(color), blue(color), alpha(color)]
bullet_color(T, color) = T[red(color), green(color), blue(color)]

function bullet_quaternion(T, rotation)
  q = Quat(rotation)
  return T[q.x, q.y, q.z, q.w]
end

bullet_vector3(T, vector) = convert(Vector{T}, vector)
end
