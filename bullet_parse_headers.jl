bullet_repo = "/Users/goretkin/repos/bullet3/"

# from bullet3/examples/pybullet/CMakeLists.txt

pybullet_srcs = (
"../../examples/SharedMemory/plugins/stablePDPlugin/SpAlg.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/SpAlg.h
../../examples/SharedMemory/plugins/stablePDPlugin/Shape.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/Shape.h
../../examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.h
../../examples/SharedMemory/plugins/stablePDPlugin/RBDModel.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/RBDModel.h
../../examples/SharedMemory/plugins/stablePDPlugin/MathUtil.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/MathUtil.h
../../examples/SharedMemory/plugins/stablePDPlugin/KinTree.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/KinTree.h
../../examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.cpp
../../examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.h
../../examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp
../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp
../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h
../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp
../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h
../../examples/SharedMemory/IKTrajectoryHelper.cpp
../../examples/SharedMemory/IKTrajectoryHelper.h
../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp
../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.h
../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp
../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.h
../../examples/OpenGLWindow/SimpleCamera.cpp
../../examples/OpenGLWindow/SimpleCamera.h
../../examples/TinyRenderer/geometry.cpp
../../examples/TinyRenderer/model.cpp
../../examples/TinyRenderer/tgaimage.cpp
../../examples/TinyRenderer/our_gl.cpp
../../examples/TinyRenderer/TinyRenderer.cpp
../../examples/SharedMemory/InProcessMemory.cpp
../../examples/SharedMemory/PhysicsClient.cpp
../../examples/SharedMemory/PhysicsClient.h
../../examples/SharedMemory/PhysicsServer.cpp
../../examples/SharedMemory/PhysicsServer.h
../../examples/SharedMemory/PhysicsServerExample.cpp
../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp
../../examples/SharedMemory/PhysicsServerSharedMemory.cpp
../../examples/SharedMemory/PhysicsServerSharedMemory.h
../../examples/SharedMemory/PhysicsDirect.cpp
../../examples/SharedMemory/PhysicsDirect.h
../../examples/SharedMemory/PhysicsDirectC_API.cpp
../../examples/SharedMemory/PhysicsDirectC_API.h
../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp
../../examples/SharedMemory/PhysicsServerCommandProcessor.h
../../examples/SharedMemory/b3PluginManager.cpp
../../examples/SharedMemory/b3PluginManager.h

../../examples/SharedMemory/PhysicsClientSharedMemory.cpp
../../examples/SharedMemory/PhysicsClientSharedMemory.h
../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp
../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h

../../examples/SharedMemory/PhysicsClientC_API.cpp
../../examples/SharedMemory/PhysicsClientC_API.h
../../examples/SharedMemory/Win32SharedMemory.cpp
../../examples/SharedMemory/Win32SharedMemory.h
../../examples/SharedMemory/PosixSharedMemory.cpp
../../examples/SharedMemory/PosixSharedMemory.h
../../examples/Utils/b3ResourcePath.cpp
../../examples/Utils/b3ResourcePath.h
../../examples/Utils/RobotLoggingUtil.cpp
../../examples/Utils/RobotLoggingUtil.h

../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp
../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h
../../examples/ThirdPartyLibs/stb_image/stb_image.cpp
../../examples/ThirdPartyLibs/stb_image/stb_image_write.cpp
../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp
../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp
../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp
../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp
../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp
../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp
../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp
../../examples/Importers/ImportURDFDemo/UrdfParser.cpp
../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp
../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp 
../../examples/MultiThreading/b3PosixThreadSupport.cpp
../../examples/MultiThreading/b3Win32ThreadSupport.cpp
../../examples/MultiThreading/b3ThreadSupportInterface.cpp
")

#pybullet_headers = map(String, filter(s->endswith(s, ".h"), split(pybullet_srcs)))
pybullet_headers_abs = joinpath.([bullet_repo],
  [
    "examples/SharedMemory/PhysicsClientC_API.h",
    "examples/SharedMemory/SharedMemoryPublic.h",
    "examples/SharedMemory/PhysicsDirectC_API.h",
    "examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.h",
  ])

pybullet_include = joinpath(bullet_repo, "examples/pybullet")
#pybullet_headers_abs = normpath.(joinpath.([pybullet_include], pybullet_headers))
bullet_includes = []
using Clang

wc = init(; headers = pybullet_headers_abs,
            output_file = joinpath(@__DIR__, "libclang_api.jl"),
            common_file = joinpath(@__DIR__, "libclang_common.jl"),
            clang_includes = vcat(pybullet_include, CLANG_INCLUDE),
            clang_args = ["-I", joinpath(pybullet_include, "..")],
            header_wrapped = (root, current)->root == current,
            header_library = x->"libclang",
            clang_diagnostics = true,
            )

run(wc)
