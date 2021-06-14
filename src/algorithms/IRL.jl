"""
# Refs
[1] F. L. Lewis, D. Vrabie, and K. Vamvoudakis,	"Reinforcement Learning and Feedback Control: Using Natural Decision Mehods to Design Optimal Adaptive Controllers," IEEE Control Systems, vol. 32, no. 6, pp.76-105, 2012.

# Variables
V̂ ∈ R: the estimate of (state) value function
    - V̂.basis: Φ
    - V̂.param: w
"""
mutable struct CTLinearIRL
    Q
    R
    V̂::LinearApproximator
    T::Real
    N::Int
end

"""
d: polynomial degree
"""
function CTLinearIRL(Q, R,
        T=0.04, N=3, d_value::Int=2, d_controller::Int=4;
        V̂=LinearApproximator(2, 2; with_bias=false),
    )
    @assert T > 0
    CTLinearIRL(Q, R, V̂, T, N)
end

function RunningCost(irl::CTLinearIRL)
    @unpack Q, R = irl
    return QuadraticCost(Q, R)
end

"""
Infer the approximate optimal input.
"""
function ApproximateOptimalInput(irl::CTLinearIRL, B)
    @unpack R = irl
    return function (x, p, t)
        P = ARE_solution(irl)
        K = inv(R) * B' * P
        û = -K * x
    end
end

function ARE_solution(irl::CTLinearIRL)
    ŵ = irl.V̂.param
    P = [  ŵ[1] ŵ[2]/2;
         ŵ[2]/2   ŵ[3]]
    P
end

function update_params_callback(irl::CTLinearIRL, tf, stop_conds)
    i = 0
    w_prev = [1, 1, 1]
    ∫rs = []
    V̂_nexts = []
    Φs = []
    stop_conds_dict = false
    affect! = function (integrator)
        @unpack t = integrator
        X = integrator.u
        @unpack x, ∫r = X
        ∫r = length(∫r) == 1 ? ∫r[1] : error("Invalid ∫r")  # for both Number and Array
        push!(∫rs, ∫r)
        push!(Φs, irl.V̂.basis(x))
        if length(∫rs) > 1
            push!(V̂_nexts, diff(∫rs[end-1:end])[1] + irl.V̂(x)[1])
        end

        # @show t, irl.V̂.param, any(values(stop_conds_dict))
        if any(values(stop_conds_dict)) == false
            if length(V̂_nexts) >= irl.N
                i += 1
                # @show hcat(Φs[end-N:end-1]...)' |> size
                # @show hcat(V̂_nexts...)' |> size
                irl.V̂.param = pinv(hcat(Φs[end-irl.N:end-1]...)') * hcat(V̂_nexts...)'
                stop_conds_dict = stop_conds(norm(irl.V̂.param-w_prev))
                w_prev = deepcopy(irl.V̂.param)
                V̂_nexts = []
                @show i, irl.V̂.param
            end
        elseif t == tf
            P = ARE_solution(irl)
            @show stop_conds_dict
            @show P
        end
    end
    # cb_train = PresetTimeCallback(0.0:irl.T:tf, affect!)
    cb_train = PeriodicCallback(affect!, irl.T, initial_affect=true)
end
