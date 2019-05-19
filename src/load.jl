# load auto-generated code

module Raw
import ..Bullet
const libclang = Bullet.c_api_lib
using CEnum
include("generated/ctypes.jl")
include("generated/libclang_common.jl")
include("generated/libclang_api.jl")
end
