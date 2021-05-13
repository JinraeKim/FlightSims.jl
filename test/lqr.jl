using FlightSims
using LinearAlgebra
using ControlSystems: lqr
using UnPack
using Plots


struct LinearSystemEnv <: AbstractEnv
    A
    B
    Q
    R
    n
    m
end
LinearSystemEnv(A, B, Q, R) = LinearSystemEnv(A, B, Q, R, size(B)...)

function State(env::LinearSystemEnv)
    () -> ones(env.n)
end

function dynamics(env::LinearSystemEnv)
    @unpack A, B = env
    return function(x, p, t; u)
        A*x + B*u
    end
end
function dynamics!(env::LinearSystemEnv)
    @unpack A, B = env
    return function(dx, x, p, t; u)
        dx .= A*x + B*u
    end
end

function test()
    n, m = 3, 3
    A = Matrix(I, n, n)
    B = Matrix(I, n, m)
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    env = LinearSystemEnv(A, B, Q, R)
    x0 = State(env)()
    K = lqr(A, B, Q, R)
    u_lqr(x, p, t) = -K*x
    # in-place
    prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_lqr);
                    tf=10.0)
    # out-of-place
    # prob, sol = sim(env, x0, apply_inputs(dynamics(env); u=u_lqr);
    #                 tf=10.0)
    df = process(env)(prob, sol)
    plot(df.times, hcat(df.states...)')
end
