using Test
using FlightSims
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra


function main(δ0=deg2rad(0), δ̇0=deg2rad(0), δ_cmd=deg2rad(10); Δt=0.01, tf=1.0)
    ζ, ω = 0.7, 150
    env = SecondOrderActuator(ζ, ω)
    x0 = State(env)(δ0, δ̇0)

    @Loggable function dynamics!(dx, x, p, t)
        (; δ, δ̇) = x
        @nested_log Dynamics!(env)(dx, x, p, t; δ_cmd=δ_cmd)
    end
    simulator = Simulator(x0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    δs = [datum.state.δ for datum in df.sol]
    δ̇s = [datum.state.δ̇ for datum in df.sol]
    δ_cmds = [datum.input for datum in df.sol]
    fig = plot(layout=(3, 1))
    plot!(fig, ts, rad2deg.(δs);
          subplot=1,
          label="δ",
         )
    plot!(fig, ts, rad2deg.(δ̇s);
          subplot=2,
          label="δ̇",
         )
    plot!(fig, ts, rad2deg.(δ_cmds);
          subplot=3,
          label="δ cmd",
         )
    display(fig)
end


@testset "SecondOrderActuator" begin
    main()
end
