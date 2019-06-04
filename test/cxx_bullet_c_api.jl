sm = Bullet.Raw.b3ConnectPhysicsDirect()

pscpid = Bullet.BulletCxx.get_physics_server_command_processor_internal_data_from_c_api_handle(sm)

# warning, the type of `dynamics_world` depends on compile-time #define `SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD`
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
