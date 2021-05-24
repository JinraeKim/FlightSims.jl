using FlightSims
using LinearAlgebra
using ControlSystems
using Random
using Plots
using Logging: global_logger # part of base
using TerminalLoggers: TerminalLogger
using Transducers: Map, withprogress


function _sample_init_cond(n)
    2*rand(n) .- 1  # ∈ [-1, 1]^n
end

function initialise()
    n, m = 3, 3
    N = 10
    A, B = Matrix(I, n, n), Matrix(I, n, m)
    Q, R = I, I
    env = LinearSystemEnv(A, B, Q, R)
    K = lqr(A, B, Q, R)
    u_explorer(x, p, t) = -K*x
    x0s = 1:N |> Map(i -> _sample_init_cond(n)) |> collect
    env, x0s, u_explorer
end


function explore(env, x0s, u_explorer)
    global_logger(TerminalLogger())  # for progress bar
    n = size(env.B)[1]
    x0 = ones(n)
    Δt, tf = 0.01, 5.0
    # probs_and_sols = x0s |> Map(x0 -> sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=tf)) |> tcollect
    probs_and_sols = x0s |> withprogress |> Map() do x0
        sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=tf)
    end |> collect
    dfs = probs_and_sols |> MapSplat((prob, sol) -> process(env)(prob, sol; Δt=Δt)) |> collect
end


function train()
end


function main(; seed=0)
    Random.seed!(seed)
    env, x0s, u_explorer = initialise()
    dfs = explore(env, x0s, u_explorer)
    train()
    p = plot()
    # dfs |> Map(df -> plot!(df.times, hcat(df.states...)')) |> tcollect
    # plot(df.times, hcat(df.states...)')
end
