"""
# Notes
`with_bias=true` provides polynomials with the degree of from 0 to `d`.

`with_bias=false` provides polynomials with the degree of `d`.

# Variables
n ∈ N: length of array, i.e., x ∈ Rⁿ
d ∈ N: degree
"""
function polynomial_basis(n::Int, d::Int; with_bias=true)
    @assert n >= 1 && d >= 0
    _n = with_bias ? n+1 : n
    exponents = multiexponents(_n, d)
    return function (x)
        _x = with_bias ? [x..., 1] : x
        exponents |> Map(exponent -> prod(_x.^exponent)) |> collect
    end
end
