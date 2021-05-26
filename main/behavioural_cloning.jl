using FlightSims
const FS = FlightSims
using LinearAlgebra
using ControlSystems
using Random
using Plots
using Logging: global_logger # part of base
using TerminalLoggers: TerminalLogger
using Transducers: Map, MapSplat, withprogress, tcollect
using Flux
using DataFrames


function _sample_init_cond(n; a=1)
    a*(2*rand(n) .- 1)  # ∈ [-a, a]^n
end

function initialise(approximator_type)
    n, m = 2, 1
    N = 100
    A, B = [0 1; 0 0], [0; 1]
    Q, R = I, I
    env = LinearSystemEnv(A, B, Q, R)
    K = lqr(A, B, Q, R)
    u_explorer(x, p, t) = -K*x
    x0s = 1:N |> Map(i -> _sample_init_cond(n)) |> collect
    û = nothing
    if approximator_type == :linear
        û = Chain(Dense(n, m))
    elseif approximator_type == :nonlinear
        û = Chain(
                  Dense(n, 16, Flux.leakyrelu),
                  Dense(16, 16, Flux.leakyrelu),
                  Dense(16, m),
                 )
    end
    env, x0s, u_explorer, û
end


function explore(env, x0s, u_explorer)
    global_logger(TerminalLogger())  # for progress bar
    n = size(env.B)[1]
    Δt, tf = 0.01, 5.0
    function process(env::LinearSystemEnv)
        return function (prob, sol; Δt=Δt)
            t0, tf = prob.tspan
            ts = t0:Δt:tf
            xs = ts |> Map(t -> sol(t)) |> collect
            us = zip(xs, ts) |> MapSplat((x, t) -> u_explorer(x, (), t)) |> collect
            DataFrame(times=ts, states=xs, actions=us)
        end
    end
    dfs = x0s |> withprogress |> Map() do x0
        prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=tf)
        df = process(env)(prob, sol; Δt=Δt)
    end |> tcollect
end


function train(û, states, actions; epochs=20)
    partition_ratio = 0.8
    data_train, data_test = FS.partitionTrainTest(zip(states, actions) |> collect, partition_ratio)  # merge and split; NEVER split data first, e.g., states_train, states_test = FS.partitionTrainTest(states)
    states_train = data_train |> Map(datum -> datum[1]) |> collect
    actions_train = data_train |> Map(datum -> datum[2]) |> collect
    states_test = data_test |> Map(datum -> datum[1]) |> collect
    actions_test = data_test |> Map(datum -> datum[2]) |> collect
    bc = BehaviouralCloning(û, states_train, actions_train)
    loss(s, a) = Flux.Losses.mse(bc.π̂(s), a)
    for epoch in 0:epochs
        println("epoch: $epoch / $epochs")
        if epoch != 0
            for batch in bc.dataloader
                FS.update!(bc, loss, batch...)
            end
        end
        training_loss = loss(hcat(states_train...), hcat(actions_train...))
        test_loss = loss(hcat(states_test...), hcat(actions_test...))
        @show training_loss
        @show test_loss
    end
    bc
end


function main(; seed=0)
    Random.seed!(seed)
    approximator_type = :nonlinear  # :linear, :nonlinear
    env, x0s, u_explorer, û = initialise(approximator_type)
    dfs = explore(env, x0s, u_explorer)
    df_merged = vcat(dfs...)
    bc = train(û, df_merged.states, df_merged.actions)
    dfs_test = explore(env, x0s, (x, p, t) -> bc.π̂(x))
    # plot
    p = plot()
    dfs_test |> Map(df -> plot!(df.times, hcat(df.states...)')) |> collect
    p
end
