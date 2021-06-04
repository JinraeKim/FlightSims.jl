"""
# References
- Reference model (e.g., xd, vd, ad, ad_dot, ad_ddot)
[1] S. J. Su, Y. Y. Zhu, H. R. Wang, and C. Yun, “A Method to Construct a Reference Model for Model Reference Adaptive Control,” Adv. Mech. Eng., vol. 11, no. 11, pp. 1–9, 2019.
# Notes
d::Int : degree
"""
struct ReferenceModelEnv <: AbstractEnv
    d::Int  # degree
    Ks::AbstractArray
end
function ReferenceModelEnv(d::Int)
    @assert d >= 0
    Ks = []
    if d == 4
        push!(Ks, Diagonal(1.0*ones(3)))
        push!(Ks, Diagonal(3.4*ones(3)))
        push!(Ks, Diagonal(5.4*ones(3)))
        push!(Ks, Diagonal(4.9*ones(3)))
        push!(Ks, Diagonal(2.7*ones(3)))
    else
        error("Assign values of matrix `Kx` manually")
    end
    ReferenceModelEnv(d, Ks)
end

"""
`x_i` denotes `i`th (time) derivative of `x`
"""
function State(env::ReferenceModelEnv)
    @unpack d = env
    return function (x0)
        xs_dict = Dict(:x_0 => x0)
        for i in 1:d
            xs_dict[Symbol(:x_, i)] = zeros(size(x0))  # e.g., xs_dict[:x1] = zeros(3)
        end
        ComponentArray((; zip(keys(xs_dict), values(xs_dict))...))
    end
end

function dynamics!(env::ReferenceModelEnv)
    @unpack d, Ks = env
    return function (dX, X, p, t; x_cmd)
        xs = [getproperty(X, Symbol(:x_, 0))]
        for i in 0:d-1
            _x_next = getproperty(X, Symbol(:x_, i+1))
            setproperty!(dX, Symbol(:x_, i), _x_next)  # e.g., dX.x0 = x1
            push!(xs, _x_next)
        end
        setproperty!(dX, Symbol(:x_, d), -sum(Ks .* xs) + Ks[1]*x_cmd)
    end
end

