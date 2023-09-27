using FlightSims
using Test
using Plots
gr()
using FSimZoo
using Convex, Mosek, MosekTools


function main()
    env = TwoDimensionalNonlinearDTSystem()
    x10, x20 = 0.5, 0.5
    x0 = State(env)(x10, x20)
    tf = 10
    simulator = Simulator(x0, apply_inputs(Dynamics!(env); u=(x, p, t) -> FSimZoo.OptimalControl(env)(x));
                          tf=tf,
                          Problem=:Discrete,
                         )
    df = solve(simulator)
    ts = df.time
    xs = [datum.state for datum in df.sol]
    us = [datum.input for datum in df.sol]
    fig_x = plot(ts, hcat(xs...)';
                 st=:scatter,
                 label=["x1" "x2"],
                 color=[:blue :red],
                 ylim=(-1, 1),
                )
    fig_u = plot(ts, hcat(us...)';
                 st=:scatter,
                 label="u",
                 ylim=(-1, 1),
                )
    fig_traj = plot(fig_x, fig_u; layout=(2, 1))
    # value functions
    x1s = -1:0.1:1
    x2s = x1s
    us = x1s
    Qs_x_fixed = [FSimZoo.OptimalQValue(env)(x0, u) for u in us]
    Q_u_fixed_func(x1, x2) = FSimZoo.OptimalQValue(env)(State(env)(x1, x2), zeros(1))
    Qs_u_and_x1_fixed = [Q_u_fixed_func(0.0, x2) for x2 in x2s]
    fig_Q = plot(x1s, x2s, Q_u_fixed_func;
                 st=:surface,
                 xlabel="x1",
                 ylabel="x2",
                )
    function min_Q_numerical(x)
        u = Convex.Variable(1)
        dx = copy(x)
        (; x1, x2) = x
        (; c, d) = env
        x1_next = c*(x1+x2)
        x2_next = c*x2 + u
        V_next = d*FSimZoo.V1(env)(x1_next) + square(x2_next)
        a = FSimZoo.CubicSolution(env)(x2)
        r = (
             (1/4)*square(square(u))
             + (3/4)*a^4 + a^2
             - d*FSimZoo.V1(env)(x1_next) + d*FSimZoo.V1(env)(x1)
             + (1-c^2)*x2^2
            )  # for V(x) = d*sqrt(abs(x1)) + x2^2
        problem = minimize(r + V_next)
        solve!(problem, () -> Mosek.Optimizer(); silent_solver=true)
        problem.optval
    end
    function min_Q_exact(x)
        u = FSimZoo.OptimalControl(env)(x)
        optval = FSimZoo.OptimalQValue(env)(x, u)
    end
    fig_min_Q_exact_minus_V = plot(
                             x1s, x2s,
                             (x1, x2) -> min_Q_exact(State(env)(x1, x2)) - FSimZoo.OptimalValue(env)(State(env)(x1, x2));
                             st=:surface,
                             zlim=(-5, 5),
                            )
    fig_min_Q_numerical_minus_V = plot(
                             x1s, x2s,
                             (x1, x2) -> min_Q_numerical(State(env)(x1, x2)) - FSimZoo.OptimalValue(env)(State(env)(x1, x2));
                             st=:surface,
                             zlim=(-5, 5),
                            )
    fig_V = plot(
                 fig_min_Q_numerical_minus_V,
                 fig_min_Q_exact_minus_V;
                 layout=(1, 2),
                )
    display(fig_traj)
end

@testset "two_dimensional_nonlinear_dt_system" begin
    main()
end
