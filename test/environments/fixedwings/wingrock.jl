using Test
using FlightSims
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra


function main(; Δt=0.01, tf=1.0)
    env = WingRock()
    x10 = 3.0
    x20 = 3.0
    X0 = State(env)(x10, x20)

    @Loggable function dynamics!(dX, X, p, t)
        (; x1, x2) = X
        ϕ = [x1, x2, abs(x1)*x2, abs(x2)*x2, x1^3]
        Δ = env.W_true' * ϕ
        u = -(x1+x2)-Δ
        if u < -5
            u = -5
        elseif u > 5
            u = 5
        end
        @nested_log Dynamics!(env)(dX, X, p, t; u=u)
    end
    simulator = Simulator(X0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    x1s = [datum.state.x1 for datum in df.sol]
    x2s = [datum.state.x2 for datum in df.sol]
    us = [datum.input for datum in df.sol]
    fig = plot(layout=(3, 1))
    plot!(fig, ts, x1s;
          subplot=1,
          label="x1",
         )
    plot!(fig, ts, x2s;
          subplot=2,
          label="x2",
         )
    plot!(fig, ts, us;
          subplot=3,
          label="u",
         )
    display(fig)
end


@testset "wingrock" begin
    main()
end
