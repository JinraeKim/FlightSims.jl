mutable struct LinearApproximator <: AbstractApproximator
    basis
    param  # parameter
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
    param = m == 1 ? init_method(w_length) : init_method(w_length, m)
    LinearApproximator(basis, param)
end

function LinearApproximator(sym::Symbol)
    if sym == :tao_bian_nonlinear_VI_V̂
        basis = PolynomialBasis(2, 2; with_bias=false)
        param = basis(rand(2)) |> length |> zeros
        return LinearApproximator(basis, param)
    elseif sym == :tao_bian_nonlinear_VI_dV̂
        @polyvar x[1:2]
        @polyvar u
        Ψ = [
             x[1], x[2],  # deg: 1
             x[1]^2, x[1]*x[2], x[2]^2,  # deg: 2
             x[1]^3, x[1]^2 * x[2], x[1] * x[2]^2, x[2]^3,  # deg: 3
             x[1]^4, x[1]^3 * x[2], x[1]^2 * x[2]^2,
             x[1] * x[2]^3, x[2]^4,  # deg: 4
             x[1]*u^3, x[2]*u^3,
            ]
        basis = PolynomialBasis(Ψ, [x..., u])
        return LinearApproximator(basis, nothing)
    end
end

(a::LinearApproximator)(x) = a.param' * a.basis(x)
