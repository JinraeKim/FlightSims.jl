using FlightSims
using Test
using Plots
gr()
using Transducers
using FSimZoo


function main()
    # c = 0.70
    # d = 0.22
    # c = 0.618
    # d = 1e-3
    # env = TwoDimensionalNonlinearDTSystem(c, d)
    env = TwoDimensionalNonlinearDTSystem()
    x10, x20 = -1.0, 1.0
    x0 = State(env)(x10, x20)
    tf = 10
    simulator = Simulator(x0, apply_inputs(Dynamics!(env); u=(x, p, t) -> FSimZoo.OptimalControl(env)(x));
                          tf=tf,
                          Problem=:Discrete,
                         )
    df = solve(simulator)
    ts = df.time
    states = df.sol |> Map(datum -> datum.state) |> collect
    inputs = df.sol |> Map(datum -> datum.input) |> collect
    xs = states
    us = inputs
    fig_x = plot(ts, hcat(xs...)';
                 st=:scatter,
                 label=["x1" "x2"],
                 color=[:blue :red],
                )
    fig_u = plot(ts, hcat(us...)';
                 st=:scatter,
                 label="u",
                )
    fig = plot(fig_x, fig_u; layout=(2, 1))
    display(fig)
    # value functions
    x1s = -1:0.01:1
    x2s = x1s
    us = x1s
    Qs_x_fixed = us |> Map(u -> FSimZoo.OptimalQValue(env)(x0, u)) |> collect
    Q_u_fixed_func(x1, x2) = FSimZoo.OptimalQValue(env)(State(env)(x1, x2), zeros(1))
    Qs_u_and_x1_fixed = x2s |> Map(x2 -> Q_u_fixed_func(0.0, x2)) |> collect
    plot(x1s, x2s, Q_u_fixed_func;
         st=:surface,
         xlabel="x1",
         ylabel="x2",
         # zlim=(-1e6, 1e6),
        )
end

@testset "two_dimensional_nonlinear_dt_system" begin
    main()
end
