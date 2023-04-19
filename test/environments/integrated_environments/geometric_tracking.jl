using Test
using FlightSims
using FSimZoo
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra


"""
[1, Sec. IV, (I)]

# References
[1] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” in 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, Dec. 2010. doi: 10.1109/CDC.2010.5717652.
"""
function main(; Δt=0.05, tf=1.0)
    multicopter = LeeQuadcopter()
    (; m, g, J) = multicopter
    X0_multicopter = State(multicopter)()
    # k_p = 5e-0
    # k_v = 5e-0
    # k_R = deg2rad(5e-0)
    # k_ω = deg2rad(5e-0)
    # τ_v, τ_a = 1e-2, 1e-2
    # controller = GeometricTrackingController(k_p=k_p, k_v=k_v, k_R=k_R, k_ω=k_ω, τ_v=τ_v, τ_a=τ_a)
    controller = GeometricTrackingController()
    X0_controller = State(controller)()
    X0 = ComponentArray(multicopter=X0_multicopter, controller=X0_controller)
    p_d = t -> [0.4*sin(0.5*pi*t), 0.6*cos(0.5*pi*t), 0.4*t]
    b_1_d = t -> [cos(0.1*pi*t), sin(0.1*pi*t), 0]
    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)
    a_d_dot = t -> ForwardDiff.derivative(a_d, t)
    a_d_ddot = t -> ForwardDiff.derivative(a_d_dot, t)
    b_1_d_dot = t -> ForwardDiff.derivative(b_1_d, t)
    b_1_d_ddot = t -> ForwardDiff.derivative(b_1_d_dot, t)

    allocator = PseudoInverseAllocator(multicopter.B)

    @Loggable function dynamics!(dX, X, params, t)
        (; p, v, R, ω) = X.multicopter
        # a = (v - X.controller.v) / controller.τ_v
        # a_dot = (a - X.controller.a) / controller.τ_a
        a = controller.ω_n_v*X.controller.z2_v
        a_dot = controller.ω_n_a*X.controller.z2_a
        ν = Command(
                    controller, p, v, R', ω;
                    a=a, a_dot=a_dot,
                    p_d=p_d(t),
                    v_d=v_d(t),
                    a_d=a_d(t),
                    a_d_dot=a_d_dot(t),
                    a_d_ddot=a_d_ddot(t),
                    b_1_d=b_1_d(t),
                    b_1_d_dot=b_1_d_dot(t),
                    b_1_d_ddot=b_1_d_ddot(t),
                    m=m, g=g, J=J,
                   )
        # f = ν[1]
        # M = ν[2:4]
        u = Command(allocator)(ν)
        # @nested_log FSimZoo.__Dynamics!(multicopter)(dX.multicopter, X.multicopter, params, t; f=f, M=M)
        @nested_log FSimZoo._Dynamics!(multicopter)(dX.multicopter, X.multicopter, params, t; u=u)
        Dynamics!(controller)(dX.controller, X.controller, params, t; v=v)
    end

    simulator = Simulator(X0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    ps = hcat([datum.state.p for datum in df.sol]...)'
    u_saturateds = hcat([datum.input.u_saturated for datum in df.sol]...)'
    p_refs = hcat([p_d(t) for t in ts]...)'
    fig = plot(layout=(2, 1))
    plot!(fig, ts, ps;
          subplot=1,
          label=["p_x" "p_y" "p_z"], lc=:blue, ls=[:solid :dash :dot])
    plot!(fig, ts, p_refs;
          subplot=1,
          label=["r_x" "r_y" "r_z"], lc=:red, ls=[:solid :dash :dot])
    plot!(fig, ts, u_saturateds;
          subplot=2,
          label=["u_1" "u_2" "u_3" "u_4"], lc=:blue, ls=[:solid :dash :dot :dashdot])
    display(fig)
end


@testset "geometric tracking" begin
    main()
end
