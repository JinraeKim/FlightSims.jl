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

function train!(env, irl; Δt=0.01, tf=10.0)
    x0 = State(env)([4.0, 0.4])
    X0 = ComponentArray(x=x0, ∫r=0.0)
    N = 3  # minibatch size
    û = FS.approximate_optimal_input(irl, env)
    augmented_dynamics! = function(env::LinearSystemEnv)
        return function (dX, X, p, t)
            @unpack x, ∫r = X
            _û = û(x, (), t)
            dynamics!(env)(dX.x, x, (), t; u=_û)
            dX.∫r = irl.running_cost(x, _û)
        end
    end
    ∫rs = []
    V̂_nexts = []
    Φs = []
    affect! = function (integrator)
        @unpack p, t = integrator
        X = integrator.u
        @unpack x, ∫r = X
        _û = û(x, p, t)
        push!(∫rs, ∫r)
        if length(∫rs) > 1
            push!(V̂_nexts, diff(∫rs[end-1:end])[1] + irl.V̂(x)[1])
            push!(Φs, irl.V̂.basis(x))
        end
        if length(Φs) >= N
            # @show hcat(Φs[end-(N-1):end]...)' |> size
            # @show hcat(V̂_nexts[end-(N-1):end]...) |> size
            irl.V̂.param = pinv(hcat(Φs[end-(N-1):end]...)') * hcat(V̂_nexts[end-(N-1):end]...)'
        end
    end
    cb_train = PresetTimeCallback(0.0:irl.T:tf, affect!)
    cb = CallbackSet(cb_train)
    # cb = CallbackSet()
    # prob, sol = sim(x0, apply_inputs(dynamics!(env); u=û); tf=tf, callback=cb)
    prob, sol = sim(X0, augmented_dynamics!(env); tf=tf, callback=cb)
    df = process(env)(prob, sol; Δt=Δt)
    xs = df.states |> Map(X -> X.x) |> collect
    # ∫rs = df.states |> Map(X -> X.∫r) |> collect
    plot(df.times, hcat(xs...)')
    plot(df.times, hcat(∫rs...)')
end

function main(; seed=1)
    Random.seed!(seed)
    env, irl = initialise()
    train!(env, irl)
end
