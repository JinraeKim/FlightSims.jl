using FlightSims
using Rotations


@testset "rotations" begin
    r = one(RotMatrix{3})
    @test euler(RotXYZ(r)) == zeros(3)  # first: Z
end
