using BinaryProvider

const verbose = ("--verbose" in ARGS)
const prefix = Prefix(joinpath(@__DIR__, "usr"))

source_archive_url = "https://github.com/goretkin/bullet3/archive/c_api_refactor_build.tar.gz"
source_archive_hash = "31dd27b6f10b6b149abbc7e5d6f5f4a683c4a6cc898fd6462f69224a18aa12b9"


products = [ LibraryProduct(prefix, ["bullet_c_api"], :bullet_c_api) ]

source_archive_path = joinpath(prefix, "downloads", "src.tar.gz")
source_unpack_path = joinpath(prefix, "downloads", "src")
build_dir = joinpath(prefix, "build")

download_verify(source_archive_url, source_archive_hash, source_archive_path)
unpack(source_archive_path, source_unpack_path)

src_dir = joinpath(source_unpack_path, "bullet3-c_api_refactor_build")
mkpath(build_dir)

cd(build_dir) do
    run(`cmake -DCMAKE_INSTALL_PREFIX=$(libdir(prefix)) -DCMAKE_POSITION_INDEPENDENT_CODE=ON $src_dir`)
    run(`make -j$(Sys.CPU_THREADS) install`)
end

write_deps_file(joinpath(@__DIR__, "deps.jl"), products, verbose=verbose)