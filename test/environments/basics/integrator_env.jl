using FlightSims
const FS = FlightSims
using LinearAlgebra
using ControlSystems: lqr
using ComponentArrays
using Transducers
using UnPack
using Plots


function test()
    n, m = 3, 3
    A = Matrix(I, n, n)
    B = Matrix(I, n, m)
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    env = LinearSystemEnv(A, B, Q, R)
    integ = FS.IntegratorEnv()
    x0 = State(env)(ones(n))
    X0 = ComponentArray(x=x0, ∫r=zeros(1))
    K = lqr(A, B, Q, R)
    function augmented_dynamics!(env::LinearSystemEnv, integ::FS.IntegratorEnv)
        return function (dX, X, p, t)
            @unpack x, ∫r = X
            u = -K*x
            r = FS.running_cost(env)(x, u)
            @show r
            @show ∫r
            dynamics!(env)(dX.x, X.x, (), t; u=u)
            dynamics!(integ)(dX.∫r, X.∫r, (), t; u=r)
        end
    end
    prob, sol = sim(X0, augmented_dynamics!(env, integ); tf=10.0)
    df = process(env)(prob, sol)
    xs = df.states |> Map(X -> X.x) |> collect
    ∫rs = df.states |> Map(X -> X.∫r) |> collect
    # plot(df.times, hcat(xs...)')
    plot(df.times, hcat(∫rs...)')
end
