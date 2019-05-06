using Cxx
using Libdl
bullet_repo = "/Users/goretkin/repos/bullet3/"
src_path = joinpath(bullet_repo, "src")

addHeaderDir(src_path, kind=C_System)
# the three libraries in bullet3/examples/HelloWorld/CMakeLists.txt
Libdl.dlopen(joinpath(bullet_repo, "build_cmake/src/Bullet3Dynamics/libBullet3Dynamics.dylib"), Libdl.RTLD_GLOBAL)
Libdl.dlopen(joinpath(bullet_repo, "build_cmake/src/Bullet3Collision/libBullet3Collision.dylib"), Libdl.RTLD_GLOBAL)
Libdl.dlopen(joinpath(bullet_repo, "build_cmake/src/LinearMath/libLinearMath.dylib"), Libdl.RTLD_GLOBAL)

cxxinclude("btBulletDynamicsCommon.h")


zv = @cxxnew btVector3(0,0,0)
ov = @cxxnew btVector3(1,2,3)

r = icxx"(*($zv)) + (*($ov));"
icxx"$(zv)->m_floats[0];"