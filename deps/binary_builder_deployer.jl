#=
I haven't worked out how to use BinaryBuilder for Bullet, because of the OpenGL dependency.
This is a stop-gap. You will have to manually deploy to GitHub.
=#

import BinaryBuilder: triplet, platform_key_abi, package
import BinaryProvider: Prefix

archive_dir = triplet(platform_key_abi())
mkpath(archive_dir)

for d = ["data", "include", "lib"]
  cp(joinpath("usr", d), joinpath(archive_dir, d), force=true)
end


package(Prefix(archive_dir), "Bullet", VersionNumber("0.0.0"))
