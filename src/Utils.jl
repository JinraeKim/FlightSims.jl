### Function approximation
"""
n ∈ N: length of array, i.e., x ∈ Rⁿ
d ∈ N: degree
"""
function polynomial_basis(n, d; with_bias=true)
    _n = with_bias ? n+1 : n
    exponents = multiexponents(_n, d)
    return function (x)
        _x = with_bias ? [x..., 1] : x
        exponents |> Map(exponent -> prod(_x.^exponent)) |> collect
    end
end

### Rotation
function euler(x::Rotations.RotXYZ)
    [x.theta1, x.theta2, x.theta3]
end
