using FlightSims
const FS = FlightSims
using DifferentialEquations
using LinearAlgebra
using Plots
using Test


function main()
    # linear system
    A = [0 1;
         0 0]  # 2 x 2
    B = [0 1]'  # 2 x 1
    n, m = 2, 1
    env = LinearSystem(A, B)  # exported from FlightSims
    x0 = State(env)([0.5, 0.5])
    # optimal control
    Q = Matrix(I, n, n)
    R = 10.0*Matrix(I, m, m)
    lqr = LQR(A, B, Q, R)  # exported from FlightSims
    u_lqr = Command(lqr)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
    u0 = u_lqr(x0)
    p0 = zeros(size(u0)...)  # auxiliary parameter

    # simulation
    Δt = 0.05
    tf = 10.0
    affect!(integrator) = integrator.p .= u_lqr(copy(integrator.u))  # auxiliary callback funciton
    cb = PeriodicCallback(affect!, Δt; initial_affect=true)  # auxiliary callback
    @Loggable function dynamics!(dx, x, p, t)
        @onlylog p  # activate this line only when logging data
        u = p
        @log x, u
        @nested_log Dynamics!(env)(dx, x, p, t; u=u)  # exported `state` and `input` from `Dynamics!(env)`
    end
    simulator = Simulator(x0, dynamics!, p0;
                          tf=tf)
    df = solve(
               simulator;
               callback=cb, savestep=Δt,
               dtmax=Δt/2,
              )
    # CAUTION: when using PeriodicCallback for MPC-like control, the initial input may be overwritten by the second input.
    # I guess that the adaptive solver "goes back in time" when `dt` is not set small enough, which violates the results with Callbacks including SavingCallback and PeriodicCallback.
    ts = df.time
    xs = [datum.x for datum in df.sol]
    us = [datum.u for datum in df.sol]
    ps = [datum.p for datum in df.sol]
    states = [datum.state for datum in df.sol]
    inputs = [datum.input for datum in df.sol]
    @test u0 == us[1]
    @test xs == states
    @test us == inputs
    p_x = plot(ts, hcat(states...)';
               title="state variable", label=["x1" "x2"], color=[:black :black], lw=1.5,
              )  # Plots
    plot!(p_x, ts, hcat(ps...)';
          ls=:dash, label="param", color=[:red :orange], lw=1.5
         )
    p_u = plot(ts, hcat(inputs...)'; title="control input", label="u", seriestype=:steppost)  # Plots
    fig = plot(p_x, p_u; layout=(2, 1))
    savefig(fig, "figures/lqr.png")
    display(fig)
    df
end

@testset "lqr example" begin
    main()
end
