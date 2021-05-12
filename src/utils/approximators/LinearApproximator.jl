mutable struct LinearApproximator <: AbstractApproximator
    basis
    w
end

"""
n: input dimension
d: polynomial degree
m: output dimension
"""
function LinearApproximator(n::Int, d::Int, m::Int=1;
        with_bias=true, init_method=rand)
    basis = PolynomialBasis(n, d; with_bias=with_bias)
    w_length = basis(rand(n)) |> length
    w = m == 1 ? init_method(w_length) : init_method(w_length, m)
    LinearApproximator(basis, w)
end

(a::LinearApproximator)(x) = a.w' * a.basis(x)
