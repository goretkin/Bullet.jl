module BulletCxx
import ..Bullet
using Cxx
using Libdl

function load_cxx_files()
  # Not sure why this is needed again. Possibly flags?
  # otherwise LLVM ERROR: Program used external function '__Z22btAlignedAllocInternalmi' which could not be resolved!
  Libdl.dlopen(Bullet.c_api_lib, Libdl.RTLD_GLOBAL)

  addHeaderDir(joinpath(Bullet.include_dir, "bullet"), kind=C_System)
  cxxinclude(joinpath(Bullet.include_dir, "bullet/btBulletDynamicsCommon.h"))


  # The following includes were added until Cxx clang stopped producing errors.

  # so that examples/SharedMemory/PhysicsServerCommandProcessor.cpp can do
  # #include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"
  addHeaderDir(joinpath(Bullet.repo_dir, "examples"), kind=C_System)

  # so that Extras/InverseDynamics/btMultiBodyTreeCreator.hpp can do
  # #include "BulletInverseDynamics/IDConfig.hpp"
  addHeaderDir(joinpath(Bullet.repo_dir, "src"), kind=C_System)

  # so that examples/SharedMemory/PhysicsServerCommandProcessor.cpp can do
  # #include "stb_image/stb_image.h"
  addHeaderDir(joinpath(Bullet.repo_dir, "examples/ThirdPartyLibs"), kind=C_System)


  cxxinclude(joinpath(Bullet.repo_dir, "examples/SharedMemory/PhysicsClientSharedMemory.cpp"))
  cxxinclude(joinpath(Bullet.repo_dir, "examples/SharedMemory/PhysicsDirect.h"))

  # PhysicsDirectInternalData is only declared here
  cxxinclude(joinpath(Bullet.repo_dir, "examples/SharedMemory/PhysicsDirect.cpp"))

  # class CommandProcessorInterface : public PhysicsCommandProcessorInterface
  # class PhysicsServerCommandProcessor : public CommandProcessorInterface
  cxxinclude(joinpath(Bullet.repo_dir, "examples/SharedMemory/PhysicsServerCommandProcessor.h"))

  # struct PhysicsServerCommandProcessorInternalData is only declared here
  cxxinclude(joinpath(Bullet.repo_dir, "examples/SharedMemory/PhysicsServerCommandProcessor.cpp"))

  # TODO set values in BulletCxx
  #const __current_compiler__ = Cxx.__current_compiler__
  #const bullet_cxx_compiler_instance = Cxx.instance(Cxx.__current_compiler__)
end

include("cxx_util.jl")
end