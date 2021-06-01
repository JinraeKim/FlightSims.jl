using FlightSims
const FS = FlightSims
using Transducers
using Plots
using Random, LinearAlgebra, ComponentArrays
using DynamicPolynomials, UnPack
using OrdinaryDiffEq, DataFrames


function initialise()
    # setting
    A, B = [-10 1; -0.002 -2], [0; 2]
    Q, R = I, 1
    env = LinearSystemEnv(A, B, Q, R)
    __x = State(env)()
    __t = 0.0
    n = length(__x)
    __u = 0.0 
    m = length(__u)
    irl = CTLinearIRL(n, m, FS.running_cost(env))
    env, irl
end

function train!(irl; w_tol=0.01, Δt=0.01, tf=10.0)
    @show irl.V̂.param
    i = 0
    w_prev = deepcopy(irl.V̂.param)
    stop_conds = function(i, w_diff_norm)
        stop_conds_dict = Dict(
                              :w_tol => w_diff_norm < w_tol,
                             )
    end
    display_res = function (irl::CTLinearIRL)
        @unpack n = irl
        @polyvar x[1:n]
        @show irl.V̂(x)
    end
		T = 4
    while true
        i += 1
				@show i
				@show irl.V̂.param
				r[i] = FS.running_cost(irl)
				Φ[i] = irl.V̂.basis

				if (mod(i-1, T) == 0 && i~=1)

					V̂[i] = r[i] - r_prev[i-T] + Ŵ' * Φ
					Φ̂[i] = Φ[end]

					V̂s = V̂[i-T:i]
					Φ̂s = Φ̂[i-T:i]

					irl.V̂.param = pinv(Φ̂s')*V̂s'
					if (mod(i-1, T*3) == 0 && i!=1)
						ŵ = irl.V̂.param
						V̂s = []
						Φ̂s = []

						P = [ŵ[1] ŵ[2]/2; ŵ[2]/2 ŵ[3]]
						K = - inv(R) * B' * P
						û = -K * x

            w_prev = deepcopy(irl.V̂.param)
					end

				end
        display_res(irl)
        stop_conds_dict = stop_conds(i, norm(irl.V̂.param-w_prev))
        if any(values(stop_conds_dict))
            @show stop_conds_dict
            break
        end
    end
    x0 = State(env)(4, 0.4)
    prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=FS.approximate_optimal_input(irl)); tf=tf)
    df = process(env)(prob, sol; Δt=Δt)
    plot(df.times, hcat(df.states...)')
end

function demonstrate(env, irl; )
end

"""
Main codes for demonstration of continuous-time linear integral reinforcement learning (CT-Linear-IRL) [1].
# References
[1] F. L. Lewis, D. Vrabie, and K. Vamvoudakis,	"Reinforcement Learning and Feedback Control: Using Natural Decision Mehods to Design Optimal Adaptive Controllers," IEEE Control Systems, vol. 32, no. 6, pp.76-105, 2012.
"""
function main()
    env, irl = initialise()
    train!(env, irl; w_tol=0.01, Δt=0.01, tf=10.0)
end
