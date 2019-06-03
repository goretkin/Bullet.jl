sm = Bullet.Raw.b3ConnectPhysicsDirect()

pd = icxx"(PhysicsDirect*) $(sm);"
@testset "PhysicsDirect isConnected" begin
  @test icxx"$(pd)->isConnected();"
end

pdid = icxx"$(pd)->m_data;"p # PhysicsDirectInternalData* m_data is a private field. Cxx allows this with the `p`

pcpi = icxx"$(pdid)->m_commandProcessor;" # PhysicsCommandProcessorInterface

pscp = icxx"(PhysicsServerCommandProcessor*) $(pcpi);"

pscpid = icxx"$(pscp)->m_data;"p # PhysicsServerCommandProcessorInternalData

# warning, the type of `dyanmics_world` depends on compile-time #define `SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD`
dynamics_world = icxx"$(pscpid)->m_dynamicsWorld;" # class btSoftMultiBodyDynamicsWorld : public btMultiBodyDynamicsWorld
# class btMultiBodyDynamicsWorld : public btDiscreteDynamicsWorld


@testset "dynamicsWorld m_gravity" begin
gravity = icxx"$(dynamics_world)->m_gravity;"p
@test [0, 0, 0] == btVector3_to_array(gravity)

Bullet.set_gravity(sm, [0.01f0, 0.02f0, -9.8f0])

gravity = icxx"$(dynamics_world)->m_gravity;"p
@test [0.01f0, 0.02f0, -9.8f0] == btVector3_to_array(gravity)
end

Bullet.Raw.b3DisconnectSharedMemory(sm);
