using Test
using FlightSims
using FSimZoo
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra
using Convex


function circleShape(h, k, r)
    θ = LinRange(0, 2*pi, 500)
    h .+ r*sin.(θ), k .+ r*cos.(θ)
end


function position_cbf(; Δt=0.05, tf=1.0)
    multicopter = GoodarziAgileQuadcopter()
    (; m, g) = multicopter
    X0_multicopter = State(multicopter)()
    controller = OuterLoopGeometricTrackingController()
    X0 = ComponentArray(
                        p=X0_multicopter.p,
                        v=X0_multicopter.v,
                       )
    p_d = t -> [0.4*sin(0.5*pi*t), 0.6*cos(0.5*pi*t), 0.4*t]
    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)

    x_c = 1.0
    y_c = 1.0
    r = 1.0
    h = p -> (p[1]-x_c)^2 + (p[2]-y_c)^2 - r^2  # >= 0
    α1 = x -> 5*x
    α2 = x -> 5*x
    cbf = InputAffinePositionCBF((p, v) -> [0, 0, g], (p, v) -> -(1/m)*I(3), h, α1, α2)

    @Loggable function dynamics!(dX, X, params, t)
        (; p, v) = X
        e3 = [0, 0, 1]
        _b_3_d = Command(
                         controller, p, v;
                         p_d=p_d(t),
                         v_d=v_d(t),
                         a_d=a_d(t),
                         m=m, g=g,
                        )
        u = Convex.Variable(length(_b_3_d))
        _b_3_d = Command(cbf, u, p, v, _b_3_d, [])
        @onlylog state = X
        @onlylog input = _b_3_d
        dX.p = v
        dX.v = g*e3 - (1/m)*_b_3_d
    end

    simulator = Simulator(X0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    ps = hcat([datum.state.p for datum in df.sol]...)'
    p_xs = hcat([datum.state.p[1] for datum in df.sol]...)'
    p_ys = hcat([datum.state.p[2] for datum in df.sol]...)'
    Fs = hcat([datum.input for datum in df.sol]...)'
    p_refs = hcat([p_d(t) for t in ts]...)'
    fig = plot(layout=(3, 1))
    plot!(fig, ts, ps;
          subplot=1,
          label=["p_x" "p_y" "p_z"], lc=:blue, ls=[:solid :dash :dot])
    plot!(fig, ts, p_refs;
          subplot=1,
          label=["r_x" "r_y" "r_z"], lc=:red, ls=[:solid :dash :dot])
    plot!(fig, ts, Fs;
          subplot=2,
          label=["F_x" "F_y" "F_z"], lc=:blue, ls=[:solid :dash :dot])
    plot!(fig, p_xs, p_ys;
          subplot=3,
          lc=:blue,
         )
    plot!(fig, circleShape(x_c, y_c, r);
          subplot=3,
          seriestype=[:shape,], lw=0.5, c=:blue, linecolor=:black, legend=false, fillalpha=0.2, aspect_ratio=1,
         )
    display(fig)
end


function position_cbf_full_dynamics(; Δt=0.05, tf=1.0)
    multicopter = GoodarziAgileQuadcopter()
    (; m, g, J) = multicopter
    ol_controller = OuterLoopGeometricTrackingController()
    il_controller = InnerLoopGeometricTrackingController()

    X0_multicopter = State(multicopter)()
    X0_il_controller = State(il_controller)()
    X0 = ComponentArray(
                        multicopter=X0_multicopter,
                        il_controller=X0_il_controller,
                       )

    p_d = t -> [0.4*sin(0.5*pi*t), 0.6*cos(0.5*pi*t), 0.4*t]
    b_1_d = t -> [cos(0.1*pi*t), sin(0.1*pi*t), 0]

    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)
    b_1_d_dot = t -> ForwardDiff.derivative(b_1_d, t)
    b_1_d_ddot = t -> ForwardDiff.derivative(b_1_d_dot, t)
    # allocation
    allocator = PseudoInverseAllocator(multicopter.B)
    # CBF
    x_c = 1.0
    y_c = 1.0
    r = 1.0
    h = p -> (p[1]-x_c)^2 + (p[2]-y_c)^2 - r^2  # >= 0
    α1 = x -> 1.0*x
    α2 = x -> 1.0*x
    cbf = InputAffinePositionCBF((p, v) -> [0, 0, g], (p, v) -> -(1/m)*I(3), h, α1, α2)

    @Loggable function dynamics!(dX, X, params, t)
        (; p, v, q, ω) = X.multicopter
        R = quat2dcm(q)
        (; z2_f, z2_f_dot) = X.il_controller
        # outer-loop
        _b_3_d = Command(
                         ol_controller, p, v;
                         p_d=p_d(t),
                         v_d=v_d(t),
                         a_d=a_d(t),
                         m=m, g=g,
                        )
        u = Convex.Variable(length(_b_3_d))
        _b_3_d = Command(cbf, u, p, v, _b_3_d, [])
        # inner-loop
        _b_3_d_dot = il_controller.ω_n_f * z2_f
        _b_3_d_ddot = il_controller.ω_n_f_dot * z2_f_dot
        ν = Command(
                    il_controller, R, ω;
                    b_1_d=b_1_d(t),
                    b_1_d_dot=b_1_d_dot(t),
                    b_1_d_ddot=b_1_d_ddot(t),
                    _b_3_d=_b_3_d,
                    _b_3_d_dot=_b_3_d_dot,
                    _b_3_d_ddot=_b_3_d_ddot,
                    J=J,
                   )
        # control allocation
        u = Command(allocator)(ν)
        @nested_log :multicopter FSimZoo._Dynamics!(multicopter)(dX.multicopter, X.multicopter, params, t; u=u)
        @log ν
        # @nested_log :multicopter FSimZoo.__Dynamics!(multicopter)(dX.multicopter, X.multicopter, params, t; f=ν[1], M=ν[2:4])
        @nested_log :il_controller Dynamics!(il_controller)(dX.il_controller, X.il_controller, params, t; f=_b_3_d)
    end

    simulator = Simulator(X0, dynamics!, []; tf=tf)
    df = solve(simulator; savestep=Δt)
    ts = df.time
    ps = hcat([datum.multicopter.state.p for datum in df.sol]...)'
    p_xs = hcat([datum.multicopter.state.p[1] for datum in df.sol]...)'
    p_ys = hcat([datum.multicopter.state.p[2] for datum in df.sol]...)'
    u_saturateds = hcat([datum.multicopter.input.u_saturated for datum in df.sol]...)'
    νs = [datum.ν for datum in df.sol]
    p_refs = hcat([p_d(t) for t in ts]...)'
    fig = plot(layout=(4, 1))
    plot!(fig, ts, ps;
          subplot=1,
          label=["p_x" "p_y" "p_z"], lc=:blue, ls=[:solid :dash :dot],
          legend=:outertopright,
         )
    plot!(fig, ts, p_refs;
          subplot=1,
          label=["r_x" "r_y" "r_z"], lc=:red, ls=[:solid :dash :dot],
          legend=:outertopright,
         )
    plot!(fig, ts, u_saturateds;
          subplot=2,
          label=["u_1" "u_2" "u_3" "u_4"], lc=:blue, ls=[:solid :dash :dot :dashdot],
          legend=:outertopright,
         )
    plot!(fig, ts, hcat([[1e-1, 1, 1, 1] .* ν for ν in νs]...)';
          subplot=3,
          label=["0.1 * f" "M_x" "M_y" "M_z"], lc=:blue, ls=[:solid :dash :dot :dashdot],
          legend=:outertopright,
         )
    plot!(fig, p_xs, p_ys;
          subplot=4,
          lc=:blue,
         )
    plot!(fig, circleShape(x_c, y_c, r);
          subplot=4,
          seriestype=[:shape,], lw=0.5, c=:blue, linecolor=:black, legend=false, fillalpha=0.2, aspect_ratio=1,
         )
    display(fig)
end


@testset "cbf" begin
    position_cbf()
    position_cbf_full_dynamics()
end
