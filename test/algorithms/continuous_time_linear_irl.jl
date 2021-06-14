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
    __t = 0.0
    irl = CTLinearIRL(Q, R)
    env, irl
end

function train!(env, irl; Δt=0.01, tf=3.0, w_tol=1e-3)
    @unpack A, B = env
    args_linearsystem = (A, B)
    linearsystem, integ = FS.LinearSystem_IntegratorEnv(args_linearsystem)  # integrated system with scalar integrator ∫r
    x0 = State(linearsystem, integ)([0.4, 4.0])
    irl.V̂.param = zeros(size(irl.V̂.param))  # zero initialisation
    û = FS.ApproximateOptimalInput(irl, B)
    _û = (X, p, t) -> û(X.x, p, t)  # for integrated system

    cb_train = FS.update_params_callback(irl, x0, tf, w_tol)
    cb = CallbackSet(cb_train)
    running_cost = FS.RunningCost(irl)
    prob, sol = sim(x0, apply_inputs(Dynamics!(linearsystem, integ, running_cost); u=_û); tf=tf, callback=cb)
    df = Process(env)(prob, sol; Δt=Δt)
    xs = df.state |> Map(X -> X.x) |> collect
    plot(df.time, hcat(xs...)')
    # ∫rs = df.state |> Map(X -> X.∫r) |> collect
    # plot(hcat(∫rs...)')
end

function main(; seed=1)
    Random.seed!(seed)
    env, irl = initialise()
    train!(env, irl; w_tol = 1e-3)
end
