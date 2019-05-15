@testset "Cxx vector operations" begin
  pv1 = @cxxnew btVector3(10,20,30)
  pv2 = @cxxnew btVector3(1,2,3)

  @test [10, 20, 30] == btVector3_ptr_to_array(pv1)

  v3 = icxx"(*($pv1)) + (*($pv2));"

  @test [11, 22, 33] == btVector3_to_array(v3)
end