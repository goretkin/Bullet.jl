# `auto r = [...]; r;` is needed otherwise it returns `Cxx.CxxCore.CppRef{Float32,(false, false, false)}`
btVector3_ptr_to_array(pv) = [icxx"auto r =$(pv)->m_floats[$i]; r;" for i = 0:2]
btVector3_to_array(v) = [icxx"auto r =$(v).m_floats[$i]; r;" for i = 0:2]