using Bullet
using Cxx
import .Bullet: Libdl

# Not sure why this is needed again. Possibly flags?
# otherwise LLVM ERROR: Program used external function '__Z22btAlignedAllocInternalmi' which could not be resolved!
Libdl.dlopen(Bullet.c_api_lib, Libdl.RTLD_GLOBAL)

addHeaderDir(joinpath(Bullet.include_dir, "bullet"), kind=C_System)
cxxinclude(joinpath(Bullet.include_dir, "bullet/btBulletDynamicsCommon.h"))


# `auto r = [...]; r;` is needed otherwise it returns `Cxx.CxxCore.CppRef{Float32,(false, false, false)}`
btVector3_ptr_to_array(pv) = [icxx"auto r =$(pv)->m_floats[$i]; r;" for i = 0:2]
btVector3_to_array(v) = [icxx"auto r =$(v).m_floats[$i]; r;" for i = 0:2]