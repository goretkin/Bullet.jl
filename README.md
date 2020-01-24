# Bullet.jl

## Code Generation
### Raw
`src/generator/bullet_parse_headers.jl` makes Julia wrappers to call the Bullet C API (`b3` functions).

e.g.

```julia
sm = Bullet.Raw.b3ConnectPhysicsDirect()
```

Warning, these functions are thin wrappers around `ccall` calls. If you pass a `[1, 1, 1]` where the Bullet API expects a `Ptr{Cdouble}`, Bullet will see `[5.0e-324, 5.0e-324, 5.0e-324]`.

### Safe
`src/generator/make_julian_api.jl` makes Julia wrappers that are safer and more Julian (using e.g. `ColorTypes`)

e.g.

```julia
sm = Bullet.Raw.b3ConnectPhysicsDirect()
command = Bullet.Raw.b3InitPhysicsParamCommand(sm)
Bullet.Safe.PhysicsParamSetGravity(command, [0, 0, -9.8])
#= equivalent to Bullet.Raw.b3PhysicsParamSetGravity(commandHandle, 0, 0, -9.8) =#
```

### C or C++
This package illustrates two different methods of interfacing with Bullet. There is the core physics code written afaik entirely in C++, and a command processor (running in its own POSIX process) that handles a special-made protocol (which could be over shared memory, or the network), and then there's C code for sending/receiving command messages. This is the way that `pybullet` works. It's C code that uses the Python C API to expose a Python interface to the C code for sending/receive command messages.

There are aspects of the C++ code that are not exposed via the command processor (after all, it's supposed to be a sort of abstraction over at least some parts of the specific engine implementation), and this package tries to demonstrate how to use `Cxx.jl` for that purpose.

Note that pybullet, taken to mean what I described above, does not do any physics calculation, and `goretkin/Bullet.jl` roughly rewrites at least part of what resembles PyBullet. Except that it's a bit better, because our ecosystem has some packages like `ColorTypes.jl`,  `Rotations.jl`, `GeometryTypes.jl` that aim to be used in interfaces. So `Bullet.jl` tries to be fancy by taking a Bullet C API function like e.g.  `setOrientation(Float x, Float y, Float z, Float w)` and turn it into `setOrientation(r::Rotation)` and then you can use whatever parameterization of rotation (quaternion, rpy, ...) you'd like.
