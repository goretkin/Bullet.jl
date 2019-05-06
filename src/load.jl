bullet_so = joinpath(@__DIR__, "..", "deps/bullet_c_api.dylib")

const libclang = bullet_so
using CEnum
include("generated/ctypes.jl")
include("generated/libclang_common.jl")
include("generated/libclang_api.jl")
