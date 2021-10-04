using FlightSims
using FSimZoo
using Test
using Plots
using Transducers

function main()
    env = TwoDimensionalNonlinearOscillator()
    x10, x20 = 1.0, 2.0
    x0 = State(env)(x10, x20)
    tf = 10.0
    simulator = Simulator(x0,
                          apply_inputs(Dynamics!(env);
                          u=(x, p, t) -> FSimZoo.OptimalControl(env)(x));
                         tf=tf)
    df = solve(simulator)
    ts = df.time
    states = df.sol |> Map(datum -> datum.state) |> collect
    x1s = states |> Map(state -> state.x1) |> collect
    x2s = states |> Map(state -> state.x2) |> collect
    fig_x1 = plot(ts, hcat(x1s...)')
    fig_x2 = plot(ts, hcat(x2s...)')
    fig = plot(fig_x1, fig_x2; layout=(2, 1))
    display(fig)
end

@testset "two_dimensional_nonlinear_oscillator" begin
    main()
end
