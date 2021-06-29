using FlightSims
using UnPack
using Test
using LinearAlgebra


function test()
    multicopter = LeeHexacopterEnv()
    @unpack m, g, B = multicopter
    allocator = PseudoInverseAllocator(B)
    ν = [m*g, zeros(3)...]
    Λ = Diagonal([0.1, 1, 1, 1, 1, 1])
    u = allocator(ν, Λ)
    @test ν ≈ B * Λ * u
end
