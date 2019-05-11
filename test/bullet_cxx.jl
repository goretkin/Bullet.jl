using Bullet

using Cxx
using Libdl

Libdl.dlopen(Bullet.c_api_lib, Libdl.RTLD_GLOBAL)

addHeaderDir(joinpath(Bullet.include_dir, "bullet"), kind=C_System)
cxxinclude(joinpath(Bullet.include_dir, "bullet/btBulletDynamicsCommon.h"))


zv = @cxxnew btVector3(0,0,0)
ov = @cxxnew btVector3(1,2,3)

r = icxx"(*($zv)) + (*($ov));"
icxx"$(zv)->m_floats[0];"