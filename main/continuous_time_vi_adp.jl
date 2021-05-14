using FlightSims
const FS = FlightSims
using Transducers
using Plots
using Random, LinearAlgebra
using DynamicPolynomials, UnPack


function initialise()
    # setting
    env = TwoDimensionalNonlinearPolynomialEnv()
    ω = 1
    u_explorer = (x, p, t) -> sin(5*t)
    __x = State(env)()
    __t = 0.0
    n = length(__x)
    __u = u_explorer(__x, (), __t) 
    m = length(__u)
    adp = CTValueIterationADP(n, m)
    adp.V̂.w = zeros(size(adp.V̂.w))  # initial guess; p.s.d.
    env, u_explorer, adp, n, m
end

function explore(dir_log, env, u_explorer; file_name="exploration.jld2")
    file_path = joinpath(dir_log, file_name) 
    prob, sol = nothing, nothing
    if isfile(file_path)
        saved_data = load(file_path)
        @unpack prob, sol = saved_data
    else
        x0 = State(env)()
        tf = 1.80  # TODO
        prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=tf)
        FlightSims.save(file_path, env, prob, sol)
    end
    df = process(env)(prob, sol; Δt=1e-2)
    df.inputs = zip(df.times, df.states) |> MapSplat((t, x) -> u_explorer(x, (), t)) |> collect
    df
end

function train!(adp, running_cost)
    @show adp.V̂.w
    i = 0
    w_prev = deepcopy(adp.V̂.w)
    stop_cond = function(i, w_diff_norm)
        i >= 100 || w_diff_norm < 0.01 
    end
    while true
        i += 1
        @show i
        lr = 1 / (i+10)
        u_norm_max = 5.0
        FS.update!(adp, running_cost, lr; u_norm_max=u_norm_max)
        @show adp.V̂.w
        display_res(adp)
        if stop_cond(i, norm(adp.V̂.w-w_prev))
            break
        else
            w_prev = deepcopy(adp.V̂.w)
        end
    end
end

function display_res(adp)
    @unpack n = adp
    @polyvar x[1:n]
    @show adp.V̂(x)
end

function main(; seed=1, dir_log="data/main/continuous_time_vi_adp")
    Random.seed!(seed)
    env, u_explorer, adp, n, m = initialise()
    df = explore(dir_log, env, u_explorer)
    FS.set_data!(adp, df)
    train!(adp, FS.running_cost(env))
    # display_res(adp)
end
