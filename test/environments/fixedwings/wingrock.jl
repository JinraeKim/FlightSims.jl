using Test
using FlightSims
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra


function test_elzebdawingrock(x10=deg2rad(6), x20=deg2rad(2); Δt=0.01, tf=1.0)
    env = ElzebdaWingRock()
    X0 = State(env)(x10, x20)

    @Loggable function dynamics!(dX, X, p, t)
        (; x1, x2) = X
        u_cmd = -5.0*(x1+x2)
        @nested_log Dynamics!(env)(dX, X, p, t; u=u_cmd)
    end
    simulator = Simulator(X0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    x1s = [datum.state.x1 for datum in df.sol]
    x2s = [datum.state.x2 for datum in df.sol]
    us = [datum.input.u_applied for datum in df.sol]
    fig = plot(layout=(3, 1))
    plot!(fig, ts, rad2deg.(x1s);
          subplot=1,
          label="x1",
         )
    plot!(fig, ts, rad2deg.(x2s);
          subplot=2,
          label="x2",
         )
    plot!(fig, ts, us;
          subplot=3,
          label="u",
         )
    display(fig)
end


function test_tarnwingrock(x10=deg2rad(6), x20=deg2rad(2); Δt=0.01, tf=1.0)
    env = TarnWingRock()
    X0 = State(env)(x10, x20)

    @Loggable function dynamics!(dX, X, p, t)
        (; x1, x2) = X

        u_cmd = -10.0*(x1+x2)
        @nested_log Dynamics!(env)(dX, X, p, t; u=u_cmd)
    end
    simulator = Simulator(X0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    x1s = [datum.state.x1 for datum in df.sol]
    x2s = [datum.state.x2 for datum in df.sol]
    us = [datum.input.u_applied for datum in df.sol]
    fig = plot(layout=(3, 1))
    plot!(fig, ts, rad2deg.(x1s);
          subplot=1,
          label="x1",
         )
    plot!(fig, ts, rad2deg.(x2s);
          subplot=2,
          label="x2",
         )
    plot!(fig, ts, us;
          subplot=3,
          label="u",
         )
    display(fig)
end


function main()
    test_elzebdawingrock()
    test_tarnwingrock()
end


@testset "wingrock" begin
    main()
end
