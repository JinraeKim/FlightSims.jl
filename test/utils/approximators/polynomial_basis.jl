using FlightSims
using DynamicPolynomials
using Test


@testset "PolynomialBasis" begin
    # automatic generation
    n = 5
    poly_basis = PolynomialBasis(n, 2)
    @show poly_basis.x
    x = rand(n)
    @show poly_basis(x)
    @show poly_basis(poly_basis.x)
    # # manual generation
    # @polyvar y[1:2]
    # poly_basis_manual = PolynomialBasis(y[1]^2 + y[2]^2, y)
    # @show poly_basis_manual
end

