using FlightSims
using TypedPolynomials
using Test


@testset "polynomial_basis" begin
    poly_basis = polynomial_basis(5, 2)
    @polyvar x[1:5]
    @show poly_basis(x)  # just for test; you don't need to use @polyvar
end

@testset "approximators" begin
    for f in readdir(@__DIR__)
        if f != splitdir(@__FILE__)[2]
            include(f)
        end
    end
end
