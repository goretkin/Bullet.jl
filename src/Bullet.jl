module Bullet

using StaticArrays
using EponymTuples
using OffsetArrays
using CoordinateTransformations

deps_file = joinpath(@__DIR__, "..", "deps", "deps.jl")
package_name = "Bullet"
if !isfile(deps_file)
  error("$(package_name) not properly installed. Please run Pkg.build(\"$(package_name)\")")
end

include(deps_file)

const data_dir = joinpath(@__DIR__, "../deps/build/data")
const c_api_lib = bullet_c_api
const include_dir = joinpath(@__DIR__, "../deps/usr/include")

include("load.jl")
include("wrap.jl")


end # module
