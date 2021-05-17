using FlightSims
const FS = FlightSims
using Transducers
using Plots
using Random, LinearAlgebra, ComponentArrays
using DynamicPolynomials, UnPack
using OrdinaryDiffEq, DataFrames


function initialise()
    # setting
    env = TwoDimensionalNonlinearPolynomialEnv()
    u_explorer = (x, p, t) -> sin(5*t)
    __x = State(env)()
    __t = 0.0
    n = length(__x)
    __u = u_explorer(__x, (), __t) 
    m = length(__u)
    adp = CTValueIterationADP(n, m)
    env, u_explorer, adp
end

function explore(dir_log, env, adp, u_explorer;
        file_name="exploration.jld2", exact_integration=true, Δt=0.003, tf=1.8)
    file_path = joinpath(dir_log, file_name) 
    prob, sol = nothing, nothing
    df = nothing
    x0 = State(env)(-2.9, -2.9)
    if exact_integration
        X0 = ComponentArray(x=x0, ∫Ψ=zeros(1, adp.N_Ψ))  # append system, 1 x N
        appended_dynamics! = function (env::TwoDimensionalNonlinearPolynomialEnv)
            return function (dX, X, p, t; u)
                dynamics!(env)(dX.x, X.x, (), t; u=u)
                dX.∫Ψ = FlightSims.Ψ(adp)(dX.x, u)
            end
        end
        prob, sol = sim(env, X0, apply_inputs(appended_dynamics!(env); u=u_explorer); tf=tf)
    else
        prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=tf)
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
                DataFrame(times=ts, states=xs, ∫Ψs=∫Ψs)
            end
        end
        df = appended_process(env)(prob, sol; Δt=Δt)
    else
        df = process(env)(prob, sol; Δt=Δt)
    end
    df.inputs = zip(df.times, df.states) |> MapSplat((t, x) -> u_explorer(x, (), t)) |> collect
    df
end

function train!(adp, running_cost; max_iter=100, w_tol=0.01)
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
        u_norm_max = 5.0
        FS.update!(adp, running_cost, lr; u_norm_max=u_norm_max)
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
    u_opt = function (x, p, t)
        # TODO
        error("Represent approximate optimal controller")
    end
    prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_opt); tf=tf)
    df = process(env)(prob, sol; Δt=Δt)
    plot(df.times, hcat(df.states...)')
end


function main(; seed=1, dir_log="data/main/continuous_time_vi_adp")
    Random.seed!(seed)
    env, u_explorer, adp = initialise()
    df = explore(dir_log, env, adp, u_explorer;
                 exact_integration=true, Δt=0.003)
    FS.set_data!(adp, df)
    train!(adp, FS.running_cost(env);
           max_iter=100, w_tol=1e-2)
    demonstrate(env, adp)
end
