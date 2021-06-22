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
    Ks::AbstractArray
    auto_diff::Bool
    x_cmd_func::Union{Function, Nothing}
end
function ReferenceModelEnv(d::Int; x_cmd_func=nothing)
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
    auto_diff = x_cmd_func == nothing ? false : true
    ReferenceModelEnv(d, Ks, auto_diff, x_cmd_func)
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

function Dynamics!(env::ReferenceModelEnv)
    @unpack d, Ks, auto_diff, x_cmd_func = env
    # derivatives for auto_diff
    funcs = nothing
    if auto_diff
        _funcs = Function[x_cmd_func]
        for i in 1:d+1
            push!(_funcs, (t) -> ForwardDiff.derivative(_funcs[i], t))
        end
    end
    funcs(t) = [_func(t) for _func in _funcs]
    @Loggable function dynamics!(dX, X, p, t; x_cmd=nothing)
        @onlylog state = X
        xs = [getproperty(X, Symbol(:x_, 0))]  # x_0, x_1, ...
        for i in 0:d-1
            _x_next = getproperty(X, Symbol(:x_, i+1))
            setproperty!(dX, Symbol(:x_, i), _x_next)  # e.g., dX.x_0 = x_1
            push!(xs, _x_next)
        end
        dx_d = nothing
        if auto_diff
            if x_cmd != nothing
                error("Do not provide a manual command for mode auto_diff=$(auto_diff)")
            end
            # dx_d = -sum(Ks .* xs) + sum([Ks..., I] .* funcs(t))
            desired_cmds = funcs(t)
            dx_d = -sum(Ks .* xs) + sum([Ks..., I] .* desired_cmds)
            @log x_cmd = desired_cmds[1]
        else
            if x_cmd == nothing
                error("Provide a manual command for mode auto_diff=$(auto_diff)")
            end
            dx_d = -sum(Ks .* xs) + Ks[1]*x_cmd
            @log x_cmd
        end
        setproperty!(dX, Symbol(:x_, d), dx_d)
    end
end
