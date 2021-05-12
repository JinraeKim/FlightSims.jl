using FlightSims
using TypedPolynomials
using Test


@testset "LinearApproximator" begin
    n, d, m = 2, 4, 3
    la = LinearApproximator(n, d, m)
    x = rand(n)
    output = la(x)
    @show output
end
