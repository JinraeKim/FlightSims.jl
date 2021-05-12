"""
# Refs
[1] T. Bian and Z.-P. Jiang,
“Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,”
in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380.
doi: 10.1109/CDC.2016.7798777.

# Variables
V̂ ∈ R: the estimate of (state) value function
dV̂ ∈ R: the estimate of time derivative of (state) value function
"""
mutable struct CTValueIterationADP
    n::Int
    m::Int
    V̂::LinearApproximator
    dV̂::LinearApproximator
    data
    ΣΦᵀΦ_inv
    Θ
end

"""
n: state dim.
m: input dim.
d: polynomial degree
"""
function CTValueIterationADP(n::Int, m::Int, d_value::Int=2, d_controller::Int=4;
        V̂=LinearApproximator(n, d_value; with_bias=false),
        dV̂=LinearApproximator(n+m, d_controller; with_bias=true),
    )
    data = DataFrame()
    ΣΦᵀΦ_inv = nothing
    Θ = nothing
    CTValueIterationADP(n, m, V̂, dV̂, data, ΣΦᵀΦ_inv, Θ)
end

"""
# Notes
adp.data contains M+1 data
j = 0, 1, ..., M-1
"""
function set_data!(adp::CTValueIterationADP, data_new)
    adp.data = data_new  # write over
    _Φ = Φ(adp)
    _Ψ = Ψ(adp)
    ts = adp.data.times  # length: M+1
    xs = adp.data.states  # length: M+1
    us = adp.data.inputs  # length: M+1
    x_js = xs[1:end-1]  # length: M
    # ΣΦᵀΦ_inv (Eq. 16)
    Φs = xs |> Map(_Φ) |> collect  # length: M+1
    Φ_js = Φs[1:end-1]  # length: M
    adp.ΣΦᵀΦ_inv = Φ_js |> Map(Φ -> Φ' * Φ) |> collect |> sum |> inv
    # Θ (Eq. 12)
    t_intervals = ts |> Partition(2; step=1) |> Map(copy) |> collect
    x_intervals = xs |> Partition(2; step=1) |> Map(copy) |> collect
    u_intervals = us |> Partition(2; step=1) |> Map(copy) |> collect
    Ψ_intervals = zip(xs, us) |> MapSplat(_Ψ) |> Partition(2; step=1) |> Map(copy) |> collect
    Θ_js = integrate.(t_intervals, Ψ_intervals)
    ΣΘᵀΘ_inv = Θ_js |> Map(Θ_j -> Θ_j' * Θ_j) |> collect |> sum |> inv
    Φ_diff_js = Φs |> diff  # length: M
    ΣΘᵀΦ_diff = zip(Θ_js, Φ_diff_js) |> MapSplat((Θ_j, Φ_diff_j) -> Θ_j' * Φ_diff_j) |> collect |> sum
    adp.Θ = ΣΘᵀΘ_inv * ΣΘᵀΦ_diff
    nothing
end

Φ(adp::CTValueIterationADP) = (x) -> reshape(adp.V̂.basis(x), 1, :)
Ψ(adp::CTValueIterationADP) = (x, u) -> reshape(adp.dV̂.basis(vcat(x, u)), 1, :)
function Ĥ(adp::CTValueIterationADP, r::Function)
    _Ψ = Ψ(adp::CTValueIterationADP)
    return (x, u, c) -> dot(_Ψ(x, u), c) + r(x, u)
end

function update!(adp::CTValueIterationADP, running_cost, lr;
        u_norm_max::Real)
    @assert u_norm_max > 0
    @unpack m, ΣΦᵀΦ_inv, data, Θ = adp
    _Ĥ = Ĥ(adp, running_cost)
    _Φ = Φ(adp)
    min_Ĥ = function(x, c)
        opt = NLopt.Opt(:LN_COBYLA, m)
        opt.min_objective = (u, grad) -> _Ĥ(x, u, c)
        opt.xtol_rel = 1e-3
        u_norm_const = function (u, grad)
            norm(u) - u_norm_max
        end
        inequality_constraint!(opt, u_norm_const)
        (minf, minx, ret) = NLopt.optimize(opt, rand(m))
    end
    xs = data.states
    term2 = xs |> Map(x -> _Φ(x)' * min_Ĥ(x, Θ*adp.V̂.w)[1]) |> collect |> sum
    adp.V̂.w += lr * ΣΦᵀΦ_inv * term2
    nothing
end
