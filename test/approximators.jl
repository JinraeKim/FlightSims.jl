using FlightSims
using TypedPolynomials


function test()
    poly_basis = polynomial_basis(5, 2)
    @polyvar x[1:5]
    @show poly_basis(x)  # just for test; you don't need to use @polyvar
    y = rand(5)
    @show poly_basis(y)
end
