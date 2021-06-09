"""
# References
- Reference model (e.g., xd, vd, ad, ad_dot, ad_ddot)
[1] S. J. Su, Y. Y. Zhu, H. R. Wang, and C. Yun, “A Method to Construct a Reference Model for Model Reference Adaptive Control,” Adv. Mech. Eng., vol. 11, no. 11, pp. 1–9, 2019.
# Notes
d: degree of the reference model
auto_diff: auto_diff = true for tracking time-varying command; otherwise for set-point regulation
x_cmd_func(t): function of time
"""
struct ReferenceModelEnv <: AbstractEnv
    d::Int  # degree
    auto_diff::Bool
    x_cmd_func::Union{Function, Nothing}
end
function ReferenceModelEnv(d::Int; x_cmd_func=nothing)
    @assert d >= 0
    auto_diff = x_cmd_func == nothing ? false : true
    ReferenceModelEnv(d, auto_diff, x_cmd_func)
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

function Params(env::ReferenceModelEnv)
    @unpack d = env
    return function (p=ComponentArray())
        if d == 4
            p = ComponentArray(
                               K_4=Diagonal(2.7*ones(3)),
                               K_3=Diagonal(4.9*ones(3)),
                               K_2=Diagonal(5.4*ones(3)),
                               K_1=Diagonal(3.4*ones(3)),
                               K_0=Diagonal(1.0*ones(3)),
                              )
        else
            error("Not defined degree $(d)")
        end
        p
    end
end

function Dynamics!(env::ReferenceModelEnv)
    @unpack d, auto_diff, x_cmd_func = env
    # derivatives for auto_diff
    funcs = nothing
    if auto_diff
        _funcs = Function[x_cmd_func]
        for i in 1:d+1
            push!(_funcs, (t) -> ForwardDiff.derivative(_funcs[i], t))
        end
    end
    funcs(t) = [_func(t) for _func in _funcs]
    return function (dX, X, p, t; x_cmd=nothing)
        xs = [getproperty(X, Symbol(:x_, 0))]
        Ks = [getproperty(p, Symbol(:K_, 0))]
        for i in 0:d-1
            _x_next = getproperty(X, Symbol(:x_, i+1))
            _p_next = getproperty(p, Symbol(:K_, i+1))
            setproperty!(dX, Symbol(:x_, i), _x_next)  # e.g., dX.x0 = x1
            push!(xs, _x_next)
            push!(Ks, _p_next)
        end
        dx_d = nothing
        if auto_diff
            if x_cmd != nothing
                error("Do not provide a manual command for auto_diff mode")
            end
            dx_d = -sum(Ks .* xs) + sum([Ks..., I] .* funcs(t))
        else
            dx_d = -sum(Ks .* xs) + Ks[1]*x_cmd
        end
        setproperty!(dX, Symbol(:x_, d), dx_d)
    end
end
