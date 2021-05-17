"""
# Refs
[1] T. Bian and Z.-P. Jiang,
“Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,”
in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380.
doi: 10.1109/CDC.2016.7798777.

# Variables
V̂ ∈ R: the estimate of (state) value function
    - V̂.basis: Φ
    - V̂.param: w
dV̂ ∈ R: the estimate of time derivative of (state) value function
    - dV̂.basis: Ψ
    - V̂.param: c (nothing)
"""
mutable struct CTValueIterationADP
    n::Int
    m::Int
    N_Ψ::Int
    V̂::LinearApproximator
    dV̂::LinearApproximator
    data
    ΣΦᵀΦ_inv
    Θ
    running_cost::Function
    u_norm_max::Real
end

"""
n: state dim.
m: input dim.
d: polynomial degree
"""
function CTValueIterationADP(n::Int, m::Int, running_cost, u_norm_max,
        d_value::Int=2, d_controller::Int=4;
        V̂=LinearApproximator(:tao_bian_nonlinear_VI_V̂),
        dV̂=LinearApproximator(:tao_bian_nonlinear_VI_dV̂),
    )
    @assert u_norm_max > 0.0
    # TODO: test with changing the function approximator, e.g., basis.
    data = DataFrame()
    ΣΦᵀΦ_inv = nothing
    Θ = nothing
    N_Ψ = length(dV̂.basis(rand(n+m)))
    CTValueIterationADP(n, m, N_Ψ, V̂, dV̂, data, ΣΦᵀΦ_inv, Θ, running_cost, u_norm_max)
end

"""
# Notes
adp.data contains M+1 data
j = 0, 1, ..., M-1
"""
function set_data!(adp::CTValueIterationADP, data_new)
    adp.data = data_new  # write over
    ts = adp.data.times  # length: M+1
    xs = adp.data.states  # length: M+1
    us = adp.data.inputs  # length: M+1
    x_js = xs[1:end-1]  # length: M
    # ΣΦᵀΦ_inv (Eq. 16)
    Φs = xs |> Map(Φ(adp)) |> collect  # length: M+1
    Φ_js = Φs[1:end-1]  # length: M
    adp.ΣΦᵀΦ_inv = Φ_js |> Map(Φ -> Φ' * Φ) |> collect |> sum |> inv
    # Θ (Eq. 12)
    Θ_js = nothing
    if isdefined(adp.data, :∫Ψs)
        # exact integration
        Θ_js = diff(adp.data.∫Ψs)
    else
        # numerical integration
        t_intervals = ts |> Partition(2; step=1) |> Map(copy) |> collect
        x_intervals = xs |> Partition(2; step=1) |> Map(copy) |> collect
        u_intervals = us |> Partition(2; step=1) |> Map(copy) |> collect
        Ψ_intervals = zip(xs, us) |> MapSplat(Ψ(adp)) |> Partition(2; step=1) |> Map(copy) |> collect
        Θ_js = integrate.(t_intervals, Ψ_intervals)
    end
    ΣΘᵀΘ_inv = Θ_js |> Map(Θ_j -> Θ_j' * Θ_j) |> collect |> sum |> inv
    Φ_diff_js = Φs |> diff  # length: M
    ΣΘᵀΦ_diff = zip(Θ_js, Φ_diff_js) |> MapSplat((Θ_j, Φ_diff_j) -> Θ_j' * Φ_diff_j) |> collect |> sum
    adp.Θ = ΣΘᵀΘ_inv * ΣΘᵀΦ_diff
    nothing
end

Φ(adp::CTValueIterationADP) = (x) -> reshape(adp.V̂.basis(x), 1, :)
Ψ(adp::CTValueIterationADP) = (x, u) -> reshape(adp.dV̂.basis(vcat(x, u)), 1, :)  # 1 × N_Ψ
function Ĥ(adp::CTValueIterationADP)
    r = adp.running_cost
    return (x, u, c) -> dot(Ψ(adp)(x, u), c) + r(x, u)
end

"""
Minimise approximate Hamiltonian.
"""
function min_Ĥ(adp::CTValueIterationADP)
    @unpack m, u_norm_max = adp
    return function (x, c)
        opt = NLopt.Opt(:LN_COBYLA, m)
        opt.min_objective = (u, grad) -> Ĥ(adp)(x, u, c)
        opt.xtol_rel = 1e-3
        u_norm_const = function (u, grad)
            norm(u) - u_norm_max
        end
        inequality_constraint!(opt, u_norm_const)
        (minf, minx, ret) = NLopt.optimize(opt, rand(m))
    end
end

function update!(adp::CTValueIterationADP, lr)
    # TODO: Add inexact way of obtaining minimums;
    # see https://github.com/JinraeKim/nonlinear-VI/blob/main/nonlinear_sys.R#L98
    @unpack m, ΣΦᵀΦ_inv, data, Θ = adp
    xs = data.states
    term2 = xs |> Map(x -> Φ(adp)(x)' * min_Ĥ(adp)(x, Θ*adp.V̂.param)[1]) |> collect |> sum
    adp.V̂.param += lr * ΣΦᵀΦ_inv * term2
    nothing
end

"""
Infer the approximate optimal input.
"""
function approximate_optimal_input(adp::CTValueIterationADP)
    # TODO: Add inexact way of obtaining approximate optimal input;
    # see https://github.com/JinraeKim/nonlinear-VI/blob/main/nonlinear_sys.R#L182
    ŵ_f = adp.V̂.param
    ĉ_f = adp.Θ * ŵ_f
    return function (x, p, t)
        û = min_Ĥ(adp)(x, ĉ_f)[2]
    end
end
