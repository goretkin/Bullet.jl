using BinDeps

@BinDeps.setup

bullet_c_api = library_dependency("bullet_c_api")
unpacked_dir = "bullet3-c_api_refactor_build"
provides(Sources, URI("https://github.com/goretkin/bullet3/archive/c_api_refactor_build.zip"), [bullet_c_api], unpacked_dir=unpacked_dir)

prefix=joinpath(BinDeps.depsdir(bullet_c_api),"usr")
src_dir = joinpath(BinDeps.depsdir(bullet_c_api), "src", unpacked_dir)
build_dir = joinpath(BinDeps.depsdir(bullet_c_api), "build")


provides(SimpleBuild,
    (@build_steps begin
        CreateDirectory(build_dir)
        GetSources(bullet_c_api)
        @build_steps begin
            ChangeDirectory(build_dir)
            `cmake -DCMAKE_INSTALL_PREFIX=$prefix -DCMAKE_POSITION_INDEPENDENT_CODE=ON $src_dir`
            `make install`
        end
end), [bullet_c_api])

@BinDeps.install Dict(:bullet_c_api => :bullet_c_api)