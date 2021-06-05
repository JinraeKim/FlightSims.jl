using FlightSims
using LinearAlgebra
using ControlSystems: lqr
using Plots


function test()
    # double integrator
    n, m = 2, 1
    A = [0 1;
         0 0]
    B = [0;
         1]
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    env = LinearSystemEnv(A, B, Q, R)  # exported from FlightSims
    K = lqr(A, B, Q, R)
    x0 = State(env)([1, 2])
    u_lqr(x, p, t) = -K*x
    prob, sol = sim(
                    x0,  # initial condition
                    apply_inputs(dynamics!(env); u=u_lqr);  # dynamics with input of LQR
                    tf=10.0,  # final time
                   )
    df = process(env)(prob, sol; Δt=0.01)  # DataFrame; `Δt` means data sampling period.
    plot(df.times, hcat(df.states...)'; label=["x1" "x2"])  # Plots
    savefig("figures/lqr.png")
end
