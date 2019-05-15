using BinaryProvider

const forcecompile = get(ENV, "JULIA_BULLET_FORCE_COMPILE", "no") == "yes"

const verbose = ("--verbose" in ARGS)
const prefix = Prefix(joinpath(@__DIR__, "usr"))

# Bullet has many URDFs, meshes, etc. We will package some of them.
const data_path = joinpath(prefix, "data")

file_products = [
    FileProduct(prefix, joinpath("data", "cube_small.urdf"), :data_cube_small_urdf),
    FileProduct(prefix, joinpath("data", "planeMesh.urdf"), :data_planeMesh_urdf),
    FileProduct(prefix, joinpath("data", "cube.obj"), :data_cube_obj),
    FileProduct(prefix, joinpath("data", "plane.obj"), :data_plane_obj),
]

products = vcat(file_products,
    [
        LibraryProduct(prefix, ["bullet_c_api"], :bullet_c_api),
    ])



source_archive_url = "https://github.com/goretkin/bullet3/archive/c_api_refactor_build.tar.gz"
source_archive_hash = "31dd27b6f10b6b149abbc7e5d6f5f4a683c4a6cc898fd6462f69224a18aa12b9"

source_archive_path = joinpath(prefix, "downloads", "src.tar.gz")
source_unpack_path = joinpath(prefix, "downloads", "src")
src_dir = joinpath(source_unpack_path, "bullet3-c_api_refactor_build")

function download()
    download_verify(source_archive_url, source_archive_hash, source_archive_path; quiet_download=true)
    unpack(source_archive_path, source_unpack_path)
end

function download_and_compile()
    download()
    build_dir = joinpath(prefix, "build")

    mkpath(build_dir)

    cd(build_dir) do
        run(`cmake -DCMAKE_INSTALL_PREFIX=$(prefix.path) -DCMAKE_POSITION_INDEPENDENT_CODE=ON $src_dir`)
        run(`make -j$(Sys.CPU_THREADS) install`)
    end

    mkpath(data_path)
    for fp in file_products
        cp(joinpath(build_dir, relpath(fp.path, prefix.path)), fp.path, force=true)
    end
end

download_info = Dict(
    MacOS(:x86_64) => ("https://github.com/goretkin/BulletBuilder/releases/download/fake1/Bullet.v0.0.0.x86_64-apple-darwin14-gcc7-cxx03.tar.gz", "9aed912e9b843b5cf75f4f5c74af43808323f06e9aaedf6671e56a265d7bc954"),
    Linux(:x86_64, libc=:glibc, compiler_abi=CompilerABI(:gcc7, :cxx11)) => ("https://github.com/goretkin/BulletBuilder/releases/download/fake2/Bullet.v0.0.0.x86_64-linux-gnu-gcc7-cxx11.tar.gz", "b5c88fa5758e80ababcfe0ab8869372c95e1ccfe7e74a10651641030c4fbe843"),
)

# Install unsatisfied or updated dependencies:
unsatisfied = any(!satisfied(p; verbose=verbose) for p in products)
if haskey(download_info, platform_key_abi()) && !forcecompile
    url, tarball_hash = download_info[platform_key_abi()]
    if !isinstalled(url, tarball_hash; prefix=prefix)
        # Download and install binaries
        install(url, tarball_hash; prefix=prefix, force=true, verbose=verbose)

        # check again whether the dependency is satisfied, which
        # may not be true if dlopen fails due to a libc++ incompatibility (#50)
        unsatisfied = any(!satisfied(p; verbose=verbose) for p in products)
    end
end

if unsatisfied || forcecompile
    download_and_compile()
end

# Cxx interface requires a bunch of forward declarations that are not installed by the Bullet build script
# (including private structures)
# download source anway.
download()

write_deps_file(joinpath(@__DIR__, "deps.jl"), products, verbose=verbose)
