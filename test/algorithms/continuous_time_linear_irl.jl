using FlightSims
const FS = FlightSims
using Transducers
using Plots
using Random, LinearAlgebra, ComponentArrays
using DynamicPolynomials, UnPack
# using OrdinaryDiffEq
using DifferentialEquations, DataFrames
using NumericalIntegration: integrate


function initialise()
    # setting
    A, B = [-10 1; -0.002 -2], [0; 2]
    Q, R = I, I
    env = LinearSystemEnv(A, B)
    n = 2
    m = 1
    __t = 0.0
    irl = CTLinearIRL(n, m, Q, R)
    env, irl
end

function train!(env, irl; Δt=0.01, tf=3.0, w_tol=1e-3)
    stop_conds = function(w_diff_norm)
        stop_conds_dict = Dict(
                               :w_tol => w_diff_norm < w_tol,
                              )
    end
    @unpack A, B = env
    args_linearsystem = (A, B)
    linearsystem, integ = FS.LinearSystem_IntegratorEnv(args_linearsystem)
    x0 = State(linearsystem, integ)([0.4, 4.0])
    irl.V̂.param = [0, 0, 0]
    û = FS.ApproximateOptimalInput(irl, B)
    _û = (X, p, t) -> û(X.x, p, t)

    cb_train = FS.update_params_callback(irl, tf, stop_conds)
    cb = CallbackSet(cb_train)
    running_cost = FS.RunningCost(irl)
    prob, sol = sim(x0, apply_inputs(Dynamics!(linearsystem, integ, running_cost); u=_û); tf=tf, callback=cb)
    df = Process(env)(prob, sol; Δt=Δt)
    xs = df.state |> Map(X -> X.x) |> collect
    # ∫rs = df.state |> Map(X -> X.∫r) |> collect
    plot(df.time, hcat(xs...)')
    # plot(hcat(∫rs...)')
end

function main(; seed=1)
    Random.seed!(seed)
    env, irl = initialise()
    train!(env, irl; w_tol = 1e-3)
end
