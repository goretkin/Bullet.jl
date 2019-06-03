using Test
using Bullet


include("demo.jl")
include("demo_robot.jl")
include("robot_collision.jl")
include("raycast.jl")


import Bullet.BulletCxx: @cxxnew, @icxx_str, btVector3_ptr_to_array, btVector3_to_array
__current_compiler__ =  Bullet.BulletCxx.Cxx.__default_compiler__ # used by Cxx macros

include("cxx_simple.jl")
include("cxx_bullet_c_api.jl")