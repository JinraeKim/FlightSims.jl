using FlightSims
using Test


@testset "PolynomialBasis" begin
    n = 5
    poly_basis = PolynomialBasis(n, 2)
    @show poly_basis.x
    x = rand(n)
    @show poly_basis(x)
    @show poly_basis(poly_basis.x)
end

