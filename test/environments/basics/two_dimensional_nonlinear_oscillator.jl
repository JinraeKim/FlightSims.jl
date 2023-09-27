using FlightSims
using Test
using Plots

function main()
    env = TwoDimensionalNonlinearOscillator()
    x10, x20 = 0.5, 0.5
    x0 = State(env)(x10, x20)
    tf = 10.0
    simulator = Simulator(x0,
                          apply_inputs(Dynamics!(env);
                          u=(x, p, t) -> FSimZoo.OptimalControl(env)(x));
                          tf=tf,
                         )
    df = solve(simulator)
    ts = df.time
    xs = [datum.state for datum in df.sol]
    us = [datum.input for datum in df.sol]
    fig_x = plot(ts, hcat(xs...)';
                 label=["x1" "x2"],
                 color=[:blue :red],
                )
    fig_u = plot(ts, hcat(us...)';
                 label="u",
                )
    fig = plot(fig_x, fig_u; layout=(2, 1))
    display(fig)
end

@testset "two_dimensional_nonlinear_oscillator" begin
    main()
end
