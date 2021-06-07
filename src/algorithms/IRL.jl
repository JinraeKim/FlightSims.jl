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
        K = inv(R) * B' * P
        û = -K * x
    end
end

function update_params_callback(irl::CTLinearIRL)
    ∫rs = []
    V̂_nexts = []
    Φs = []
    stop_conds_dict = []
    affect! = function (integrator)
        @unpack p, t = integrator
        X = integrator.u
        @unpack x, ∫r = X
        _û = û(x, p, t)
        push!(∫rs, ∫r)
        push!(Φs, irl.V̂.basis(x))
        if length(∫rs) > 1
            push!(V̂_nexts, diff(∫rs[end-1:end])[1] + irl.V̂(x)[1])
        end

        # @show t, irl.V̂.param, any(values(stop_conds_dict))
        if any(values(stop_conds_dict)) == false
            if length(V̂_nexts) >= N
                i += 1
                # @show hcat(Φs[end-N:end-1]...)' |> size
                # @show hcat(V̂_nexts...)' |> size
                irl.V̂.param = pinv(hcat(Φs[end-N:end-1]...)') * hcat(V̂_nexts...)'
                stop_conds_dict = stop_conds(norm(irl.V̂.param-w_prev))
                w_prev = deepcopy(irl.V̂.param)
                V̂_nexts = []
                @show i, irl.V̂.param
            end
        elseif t == tf
            P = [  irl.V̂.param[1] irl.V̂.param[2]/2;
                 irl.V̂.param[2]/2   irl.V̂.param[3]]
            @show stop_conds_dict
            @show P
        end
    end
    cb_train = PresetTimeCallback(0.0:irl.T:tf, affect!)
end
