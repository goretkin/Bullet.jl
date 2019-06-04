# `auto r = [...]; r;` is needed otherwise it returns `Cxx.CxxCore.CppRef{Float32,(false, false, false)}`
btVector3_ptr_to_array(pv) = [icxx"auto r =$(pv)->m_floats[$i]; r;" for i = 0:2]
btVector3_to_array(v) = [icxx"auto r =$(v).m_floats[$i]; r;" for i = 0:2]

function get_physics_server_command_processor_internal_data_from_c_api_handle(sm)
  pd = icxx"(PhysicsDirect*) $(sm);"
  is_connected = icxx"$(pd)->isConnected();"
  @assert is_connected

  pdid = icxx"$(pd)->m_data;"p # PhysicsDirectInternalData* m_data is a private field. Cxx allows this with the `p`

  pcpi = icxx"$(pdid)->m_commandProcessor;" # PhysicsCommandProcessorInterface

  pscp = icxx"(PhysicsServerCommandProcessor*) $(pcpi);"

  pscpid = icxx"$(pscp)->m_data;"p # PhysicsServerCommandProcessorInternalData
end

function get_collider(sm, body_id, link_id)
  # copy logic from PhysicsServerCommandProcessor::processCollisionFilterCommand
  m_data = get_physics_server_command_processor_internal_data_from_c_api_handle(sm)
  body = icxx"$m_data->m_bodyHandles.getHandle($body_id);"

  collider = if (icxx"$body->m_multiBody;" != C_NULL)
    if (link_id == Bullet.LINK_ID_NONE)
      icxx"$body->m_multiBody->getBaseCollider();"
    else
      if (link_id >= 0 && link_id < icxx"$body->m_multiBody->getNumLinks();")
        icxx"$body->m_multiBody->getLinkCollider($link_id);"
      end
    end
  else
    if (icxx"$body->m_rigidBody;" != C_NULL)
      icxx"$body->m_rigidBody;"
    end
  end

  return collider
end

function get_collision_flags(collider)
  collision_filter_group = icxx"$collider->getBroadphaseHandle()->m_collisionFilterGroup;"
  collision_filter_mask = icxx"$collider->getBroadphaseHandle()->m_collisionFilterMask;"

  return (group=collision_filter_group, mask=collision_filter_mask)
end