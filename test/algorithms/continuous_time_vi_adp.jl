using FlightSims
const FS = FlightSims
using Transducers
using Plots
using Random, LinearAlgebra, ComponentArrays
using DynamicPolynomials, UnPack
using DifferentialEquations, DataFrames


function initialise()
    # setting
    env = TwoDimensionalNonlinearPolynomialEnv()
    u_explorer = (x, p, t) -> sin(5*t)
    __x = State(env)()
    __t = 0.0
    n = length(__x)
    __u = u_explorer(__x, (), __t) 
    m = length(__u)
    u_norm_max = 5.0  # input constraint for numerical optimisation; TODO: can be deprecated
    adp = CTValueIterationADP(n, m, FS.RunningCost(env), u_norm_max)
    env, u_explorer, adp
end

function explore(dir_log, env, adp, u_explorer;
        file_name="exploration.jld2",
        exact_integration=true, Δt=0.003, tf=1.8,  # the settings in the paper and implemented code by T. Bian
    )
    file_path = joinpath(dir_log, file_name) 
    prob, sol = nothing, nothing
    df = nothing
    x0 = State(env)(-2.9, -2.9)  # the setting in the paper
    if exact_integration
        X0 = ComponentArray(x=x0, ∫Ψ=zeros(1, adp.N_Ψ))  # append system, 1 x N
        appended_dynamics! = function (env::TwoDimensionalNonlinearPolynomialEnv)
            return function (dX, X, p, t; u)
                Dynamics!(env)(dX.x, X.x, (), t; u=u)
                dX.∫Ψ = FlightSims.Ψ(adp)(dX.x, u)
            end
        end
        prob, sol = sim(X0, apply_inputs(appended_dynamics!(env); u=u_explorer); tf=tf)
    else
        prob, sol = sim(x0, apply_inputs(Dynamics!(env); u=u_explorer); tf=tf)
    end
    FlightSims.save(file_path, env, prob, sol)
    if exact_integration
        appended_process = function (env::TwoDimensionalNonlinearPolynomialEnv)
            return function (prob::ODEProblem, sol::ODESolution; Δt)
                t0, tf = prob.tspan
                ts = t0:Δt:tf
                Xs = ts |> Map(t -> sol(t)) |> collect
                xs = Xs |> Map(X -> X.x) |> collect
                ∫Ψs = Xs |> Map(X -> X.∫Ψ) |> collect
                DataFrame(time=ts, state=xs, ∫Ψs=∫Ψs)
            end
        end
        df = appended_process(env)(prob, sol; Δt=Δt)
    else
        df = Process(env)(prob, sol; Δt=Δt)
    end
    df.input = zip(df.time, df.state) |> MapSplat((t, x) -> u_explorer(x, (), t)) |> collect
    df
end

function train!(adp; max_iter=100, w_tol=0.01)
    @show adp.V̂.param
    i = 0
    w_prev = deepcopy(adp.V̂.param)
    stop_conds = function(i, w_diff_norm)
        stop_conds_dict = Dict(
                              :max_iter => i >= max_iter,
                              :w_tol => w_diff_norm < w_tol,
                             )
    end
    display_res = function (adp::CTValueIterationADP)
        @unpack n = adp
        @polyvar x[1:n]
        @show adp.V̂(x)
    end
    while true
        i += 1
        @show i
        lr = 1 / (i+10)
        FS.update!(adp, lr)
        @show adp.V̂.param
        display_res(adp)
        stop_conds_dict = stop_conds(i, norm(adp.V̂.param-w_prev))
        if any(values(stop_conds_dict))
            @show stop_conds_dict
            break
        else
            w_prev = deepcopy(adp.V̂.param)
        end
    end
end

function demonstrate(env, adp; Δt=0.01, tf=10.0)
    x0 = State(env)(-2.9, -2.9)
    prob, sol = sim(x0, apply_inputs(Dynamics!(env); u=FS.approximate_optimal_input(adp)); tf=tf)
    df = Process(env)(prob, sol; Δt=Δt)
    plot(df.time, hcat(df.state...)')
end


"""
Main codes for demonstration of continuous-time value-iteration adaptive dynamic programming (CT-VI-ADP) [1].
# References
[1] T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.
[2] Forked repo of the code implementation by T. Bian, https://github.com/JinraeKim/nonlinear-VI/blob/main/nonlinear_sys.R

# Settings
`exact_integration`:
if `true`, `∫Ψ` is calculated with the dynamical system simultaneously (i.e., regarded as the true solution).
if `false`, it is calculated by numerical integration (default setting may be trapezoidal method.)

`Δt`: data sampling period
"""
function main(; seed=1, dir_log="data/main/continuous_time_vi_adp")
    Random.seed!(seed)
    env, u_explorer, adp = initialise()
    df = explore(dir_log, env, adp, u_explorer;
                 exact_integration=true, Δt=0.003)
    FS.set_data!(adp, df)
    train!(adp;
           max_iter=100, w_tol=1e-2)
    demonstrate(env, adp)
end
