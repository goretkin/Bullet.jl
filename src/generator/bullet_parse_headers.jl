using Clang

package_dir = joinpath(@__DIR__, "..", "..")
bullet_repo = joinpath(package_dir, "deps/usr/downloads/src/bullet3-c_api_refactor_build")
generated_dir = joinpath(package_dir, "src", "generated")

pybullet_headers_abs = joinpath.([bullet_repo],
  [
    "examples/SharedMemory/PhysicsClientC_API.h",
    "examples/SharedMemory/SharedMemoryPublic.h",
    "examples/SharedMemory/PhysicsDirectC_API.h",
    "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.h",
  ])

pybullet_include = joinpath(bullet_repo, "examples/pybullet")


wc = init(; headers = pybullet_headers_abs,
            output_file = joinpath(generated_dir, "libclang_api.jl"),
            common_file = joinpath(generated_dir, "libclang_common.jl"),
            clang_includes = vcat(pybullet_include, CLANG_INCLUDE),
            clang_args = ["-I", joinpath(pybullet_include, "..")],
            header_wrapped = (root, current)->root == current,
            header_library = x->"libclang",
            clang_diagnostics = true,
            )

run(wc)
