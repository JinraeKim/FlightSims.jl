using Test
using FlightSims
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra


function main(δ=deg2rad(10); Δt=0.01, tf=1.0)
    missile = MissileLongitudinal()
    x0 = State(missile)(0, 0, 4, 0, 10_000)

    @Loggable function dynamics!(dx, x, p, t)
        @nested_log Dynamics!(missile)(dx, x, p, t; δ=δ)
    end
    simulator = Simulator(x0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    αs = [datum.state.α for datum in df.sol]
    δs = [datum.input for datum in df.sol]
    fig = plot(layout=(2, 1))
    plot!(fig, ts, rad2deg.(αs);
          subplot=1,
          label="α",
         )
    plot!(fig, ts, rad2deg.(δs);
          subplot=2,
          label="δ",
         )
    display(fig)
end


@testset "MissileLongitudinal" begin
    main()
end
