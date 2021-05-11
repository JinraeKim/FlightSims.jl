using FlightSims
using TypedPolynomials
using Test


@testset "LinearApproximator" begin
    n, d = 2, 4
    la = LinearApproximator(n, d)
    x = rand(n)
    output = la(x)
    @show output
end
