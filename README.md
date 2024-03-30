# Bullet.jl

## Code Generation
### Raw
`src/generator/bullet_parse_headers.jl` makes Julia wrappers to call the Bullet C API (`b3` functions).

e.g.

```julia
sm = Bullet.Raw.b3ConnectPhysicsDirect()
```

Warning, these functions are thin wrappers around `ccall` calls. If you pass a `[1, 1, 1]` where the Bullet API expects a `Ptr{Cdouble}`, Bullet will see `[5.0e-324, 5.0e-324, 5.0e-324]` (binary representation of integers interpreted as floats. Also, I don't remember what happens if you pass in an array with different byte size).

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
This package illustrates two different methods of interfacing with Bullet, because "Bullet" has multiple interfaces.
- There is the core physics code written entirely in C++.
- There is the command processor, which runs in its own POSIX process, that handles a special-made protocol (which could be over shared memory, or the network), and calls the core code above.
- and then there is C code for sending/receiving command messages.

The popular library `pybullet` uses the last layer. The `pybullet` project is C code that uses the Python C API to expose a Python interface to the C code for sending/receive command messages.

There are aspects of the C++ code that are not exposed via the command processor (after all, it's supposed to be a sort of abstraction over at least some parts of the specific engine implementation), and this package tries to demonstrate how to use `Cxx.jl` for the purpose of accessing the physics engine directly.

Note that pybullet, taken to mean what I described above, does not do any physics calculation, and `goretkin/Bullet.jl` roughly rewrites at least part of what resembles PyBullet. Except that it's a bit better, because our ecosystem has some packages like `ColorTypes.jl`,  `Rotations.jl`, `GeometryTypes.jl` that aim to be used in interfaces. `Bullet.jl` wraps both a Bullet C API function, e.g.  `setOrientation(Float x, Float y, Float z, Float w)`, directly, and also exposes an interface with richer data types, e.g. `setOrientation(r::Rotation)`, allowing you to easily use whatever parameterization of rotation (quaternion, rpy, ...) you'd like in the interface (the rotation quantity will be converted to a unit quaternion).
