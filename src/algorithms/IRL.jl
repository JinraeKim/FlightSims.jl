"""
# Refs
[1] F. L. Lewis, D. Vrabie, and K. Vamvoudakis,	"Reinforcement Learning and Feedback Control: Using Natural Decision Mehods to Design Optimal Adaptive Controllers," IEEE Control Systems, vol. 32, no. 6, pp.76-105, 2012.

# Variables
V̂ ∈ R: the estimate of (state) value function
    - V̂.basis: Φ
    - V̂.param: w
"""
mutable struct CTLinearIRL
    n::Int
    m::Int
    V̂::LinearApproximator
    running_cost::Function
    T::Real
end

"""
n: state dim.
m: input dim.
d: polynomial degree
"""
function CTLinearIRL(n::Int, m::Int, running_cost,
        T=0.04, d_value::Int=2, d_controller::Int=4;
        V̂=LinearApproximator(2, 2; with_bias=false),
    )
    @assert T > 0
    CTLinearIRL(n, m, V̂, running_cost, T)
end

"""
Infer the approximate optimal input.
"""
function approximate_optimal_input(irl::CTLinearIRL, env::LinearSystemEnv)
    @unpack B, R = env
    return function (x, p, t)
        ŵ = irl.V̂.param
        P = [  ŵ[1] ŵ[2]/2;
             ŵ[2]/2   ŵ[3]]
        K = - inv(R) * B' * P
        û = -K * x
    end
end
