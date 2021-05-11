using FlightSims
using TypedPolynomials
using Test


@testset "approximators" begin
    poly_basis = polynomial_basis(5, 2)
    @polyvar x[1:5]
    @show poly_basis(x)  # just for test; you don't need to use @polyvar
end
