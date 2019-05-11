module Bullet

using StaticArrays
using EponymTuples
using OffsetArrays
using CoordinateTransformations

const data_dir = joinpath(@__DIR__, "../deps/build/data")
const c_api_lib = joinpath(@__DIR__, "../deps/usr/lib/bullet_c_api.dylib")
const include_dir = joinpath(@__DIR__, "../deps/usr/include")

include("load.jl")
include("wrap.jl")


end # module
