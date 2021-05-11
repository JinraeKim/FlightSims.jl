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
struct CTValueIterationADP
    V̂::LinearApproximator
    dV̂::LinearApproximator
    data
    ΣΦᵀΦ_inv
end

"""
n: state dim.
m: input dim.
d: polynomial degree
"""
function CTValueIterationADP(n::Int, m::Int, d_value::Int=2, d_controller::Int=4;
        with_bias_value=false, with_bias_controller=true,
    )
    V̂ = LinearApproximator(n, d_value; with_bias=with_bias_value)
    dV̂ = LinearApproximator(n+m, d_controller; with_bias=with_bias_controller)
    data = DataFrame()
    ΣΦᵀΦ_inv = nothing
    CTValueIterationADP(V̂, dV̂, data, ΣΦᵀΦ_inv)
end

function set_data!(adp::CTValueIterationADP)
    @unpack data, V̂, ΣΦᵀΦ_inv = adp
    return function (data_new)
        data = data_new
        xs = data.states
        Φ = V̂.basis
        ΦᵀΦ_js = xs |> Map(x -> Φ(x)' * Φ(x)) |> collect
        ΣΦᵀΦ_inv = ΦᵀΦ_js |> sum |> inv
        nothing
    end
end

Φ(adp::CTValueIterationADP) = (x) -> adp.V̂.basis(x)
Ψ(adp::CTValueIterationADP) = (x, u) -> adp.dV̂.basis(vcat(x, u))
function Ĥ(adp::CTValueIterationADP, r::Function)
    Ψ = Ψ(adp::CTValueIterationADP)
    return (x, u, c) -> Ψ(x, u)*c + r(x, u)
end

function update!(adp::CTValueIterationADP, lr)
    @unpack V, ΣΦᵀΦ_inv = adp
    error("TODO")
    xs |> Map(x -> Φ(x)' * min(Ĥ)) |> collect
    # V.w += lr * ΣΦᵀΦ_inv *
end
