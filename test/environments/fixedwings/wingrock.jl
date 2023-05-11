using Test
using FlightSims
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra


function main(; Δt=0.01, tf=1.0)
    env = WingRock()
    x10, x20 = 0.3, 0.0
    X0 = State(env)(x10, x20)

    function dynamics!(dX, X, p, t)
        (; x1, x2) = X
        Dynamics!(env)(dX, X, p, t; u=-(x1+x2))
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
          subplot=1,
          label="x2",
         )
    plot!(fig, ts, us;
          subplot=1,
          label="u",
         )
    display(fig)
end


@testset "wingrock" begin
    main()
end
