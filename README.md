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
