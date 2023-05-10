using Test
using FlightSims
using FSimZoo
using ForwardDiff
using Plots
using ComponentArrays
using LinearAlgebra
using Convex
using ECOS
using DiffEqCallbacks


struct Ellipse
    xc
    yc
    a
    b
end

struct Circle
    xc
    yc
    r
end


function shape(obs::Ellipse)
    (; xc, yc, a, b) = obs
    θ = LinRange(0, 2*pi, 250)
    xc .+ a*sin.(θ), yc .+ b*cos.(θ)
end

function shape(obs::Circle)
    (; xc, yc, r) = obs
    θ = LinRange(0, 2*pi, 500)
    xc .+ r*sin.(θ), yc .+ r*cos.(θ)
end


function generate_h(obs::Ellipse)
    (; xc, yc, a, b) = obs
    p -> ((p[1]-xc)/a)^2 + ((p[2]-yc)/b)^2 - 1.0^2  # >= 0
end

function generate_h(obs::Circle)
    (; xc, yc, r) = obs
    p -> ((p[1]-xc)/r)^2 + ((p[2]-yc)/r)^2 - 1.0^2  # >= 0
end


function position_cbf(; Δt=0.01, Δt_save=0.05, tf=1.0)
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

    obstacle = Circle(1.0, 1.0, 1.0)
    h = generate_h(obstacle)
    α1 = x -> 5*x
    α2 = x -> 5*x
    cbf = InputAffinePositionCBF((p, v) -> [0, 0, g], (p, v) -> -(1/m)*I(3), h, α1, α2)

    @Loggable function dynamics!(dX, X, params, t)
        (; p, v) = X
        e3 = [0, 0, 1]
        _b_3_d = params

        @onlylog state = X
        @onlylog input = _b_3_d
        dX.p = v
        dX.v = g*e3 - (1/m)*_b_3_d
    end

    params0 = zeros(3)
    simulator = Simulator(X0, dynamics!, params0; tf=tf)
    function affect!(integrator)
        X = copy(integrator.u)
        t = copy(integrator.t)
        (; p, v) = X
        _b_3_d = Command(
                         controller, p, v;
                         p_d=p_d(t),
                         v_d=v_d(t),
                         a_d=a_d(t),
                         m=m, g=g,
                        )
        u = Convex.Variable(length(_b_3_d))
        _b_3_d = Command(cbf, u, p, v, _b_3_d, [])

        integrator.p = _b_3_d
    end
    cb = PeriodicCallback(affect!, Δt; initial_affect=true)
    df = solve(simulator; callback=cb, savestep=Δt_save)
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
    plot!(fig, shape(obstacle);
          subplot=3,
          seriestype=[:shape,], lw=0.5, c=:blue, linecolor=:black, legend=false, fillalpha=0.2, aspect_ratio=1,
         )
    display(fig)
end


"""
# Refs
[1] M. Faessler, A. Franchi, and D. Scaramuzza, “Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories,” IEEE Robot. Autom. Lett., vol. 3, no. 2, pp. 620–626, Apr. 2018, doi: 10.1109/LRA.2017.2776353.
"""
function position_cbf_full_dynamics(;
        Δt=0.01, amp_for_plot=10,
        tf=1.0,
        # tf=15.0,
        func_type=:pma, T=1e-1,
        D=diagm([0.25, 0.50, 0]),
        # D=diagm(zeros(3)),
    )
    multicopter = GoodarziAgileQuadcopter(D=D)
    (; m, g, J) = multicopter
    ol_controller = OuterLoopGeometricTrackingController(k_p=3.0, k_v=3.0)
    il_controller = InnerLoopGeometricTrackingController(k_R=0.7, k_ω=0.12)

    p0 = [-0.0, -0.0, -1.0]
    X0_multicopter = State(multicopter)(p0)
    X0_il_controller = State(il_controller)()
    X0 = ComponentArray(
                        multicopter=X0_multicopter,
                        il_controller=X0_il_controller,
                       )

    # p_d = t -> [0.4*sin(0.5*pi*t), 0.6*cos(0.5*pi*t), 0.4*t]
    # b_1_d = t -> [cos(0.1*pi*t), sin(0.1*pi*t), 0]
    p_d = function (t)
        # p = [0.2*t, 0.02*t^2 + 0.05*t, p0[3]]
        p = [3-3*cos(0.5*t), 3*sin(0.5*t), p0[3]]
        return p
    end  # for tf = 10
    b_1_d = t -> [1, 0, 0]
    # b_1_d = t -> [cos(t), sin(t), 0]

    v_d = t -> ForwardDiff.derivative(p_d, t)
    a_d = t -> ForwardDiff.derivative(v_d, t)
    b_1_d_dot = t -> ForwardDiff.derivative(b_1_d, t)
    b_1_d_ddot = t -> ForwardDiff.derivative(b_1_d_dot, t)
    # allocation
    allocator = PseudoInverseAllocator(multicopter.B)
    # CBF
    obstacles = [
                 Ellipse(+3.0, +2.8, 0.50, 0.40),
                 Ellipse(+2.5, -2.4, 0.50, 0.80),
                 # Ellipse(+5.0, -3.8, 0.40, 1.00),
                 # Circle(-0.1, -0.5, 0.25),
                ]
    hs = [generate_h(obs) for obs in obstacles]
    α1 = x -> 5.0*x
    α2 = x -> 7.5*x
    cbfs = [InputAffinePositionCBF((p, v) -> [0, 0, g], (p, v) -> -(1/m)*I(3), h, α1, α2) for h in hs]

    @Loggable function dynamics!(dX, X, params, t)
        (; p, v, R, ω) = X.multicopter
        (; z2_f, z2_f_dot) = X.il_controller
        _b_3_d = params
        # inner-loop
        _b_3_d_dot = il_controller.ω_n_f * z2_f
        _b_3_d_ddot = il_controller.ω_n_f_dot * z2_f_dot
        ν = Command(
                    il_controller, R', ω;
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
        @log _b_3_d
        # @nested_log :multicopter FSimZoo.__Dynamics!(multicopter)(dX.multicopter, X.multicopter, params, t; f=ν[1], M=ν[2:4])
        @nested_log :il_controller Dynamics!(il_controller)(dX.il_controller, X.il_controller, params, t; f=_b_3_d)
    end

    function affect!(integrator)
        X = copy(integrator.u)
        t = copy(integrator.t)
        (; p, v) = X.multicopter
        # outer-loop
        _b_3_d = Command(
                         ol_controller, p, v;
                         p_d=p_d(t),
                         v_d=v_d(t),
                         a_d=a_d(t),
                         m=m, g=g,
                        )
        _b_3_d_cvx = Convex.Variable(length(_b_3_d))
        if func_type == :pma
            constraints = [FSimZoo.generate_constraint(cbf, p, v, _b_3_d_cvx) for cbf in cbfs]  # TODO: conventional high-order CBF methods
        elseif func_type == :plse
            _constraints = [FSimZoo._generate_constraint(cbf, p, v) for cbf in cbfs]
            u_coeff = vcat([tmp[1] for tmp in _constraints]...)
            bias = vcat([tmp[2] for tmp in _constraints]...)
            lse_constraint = T*Convex.logsumexp(-(u_coeff*_b_3_d_cvx + bias)/T) <= 0.0
            constraints = [lse_constraint]
        end

        if length(constraints) == 0
            prob = minimize(sumsquares(_b_3_d_cvx-_b_3_d))
        else
            prob = minimize(sumsquares(_b_3_d_cvx-_b_3_d), constraints)
        end
        solve!(prob, ECOS.Optimizer; silent_solver=true)
        _b_3_d = reshape(_b_3_d_cvx.value, size(_b_3_d)...)

        integrator.p = _b_3_d
    end

    params0 = [0, 0, m*g]
    simulator = Simulator(X0, dynamics!, params0, tf=tf)
    cb = PeriodicCallback(affect!, Δt; initial_affect=true)
    df = @time solve(simulator; callback=cb, savestep=Δt)
    df_time = df.time[1:amp_for_plot:end]
    df_sol = df.sol[1:amp_for_plot:end]
    ts = df_time
    ps = [datum.multicopter.state.p for datum in df_sol]
    p_xs = hcat([datum.multicopter.state.p[1] for datum in df_sol]...)'
    p_ys = hcat([datum.multicopter.state.p[2] for datum in df_sol]...)'
    u_saturateds = hcat([datum.multicopter.input.u_saturated for datum in df_sol]...)'
    νs = [datum.ν for datum in df_sol]
    p_refs = [p_d(t) for t in ts]
    p_ref_xs = [p[1] for p in p_refs]
    p_ref_ys = [p[2] for p in p_refs]
    fig = plot(layout=(2, 2))
    plot!(fig, ts, hcat(ps...)';
          subplot=1,
          label=["p_x" "p_y" "p_z"], lc=:blue, ls=[:solid :dash :dot],
          legend=:outertopright,
         )
    plot!(fig, ts, hcat(p_refs...)';
          subplot=1,
          label=["r_x" "r_y" "r_z"], lc=:red, ls=[:solid :dash :dot],
          legend=:outertopright,
         )
    plot!(fig, ts, u_saturateds;
          subplot=2,
          label=["u_1" "u_2" "u_3" "u_4"], lc=:blue, ls=[:solid :dash :dot :dashdot],
          legend=:outertopright,
         )
    # plot!(fig, ts, hcat([[1e-1, 1, 1, 1] .* ν for ν in νs]...)';
    #       subplot=3,
    #       label=["0.1 * f" "M_x" "M_y" "M_z"], lc=:blue, ls=[:solid :dash :dot :dashdot],
    #       legend=:outertopright,
    #      )
    for (i, h) in enumerate(hs)
        plot!(fig, ts, [h(p) for p in ps];
              label="h_$(i)",
              subplot=3,
             )
    end
    plot!(fig, p_xs, p_ys;
          subplot=4,
          lc=:blue,
          label="",
         )
    plot!(fig, p_ref_xs, p_ref_ys;
          subplot=4,
          lc=:red,
          ls=:dash,
          label="",
         )
    for obs in obstacles
        plot!(fig, shape(obs);
              subplot=4,
              seriestype=[:shape,], lw=0.5, c=:blue, linecolor=:black, legend=false, fillalpha=0.2, aspect_ratio=1,
              label="",
             )
    end
    # fig = plot()
    # plot!(fig, p_xs, p_ys;
    #       lc=:blue,
    #      )
    # plot!(fig, p_ref_xs, p_ref_ys;
    #       lc=:red,
    #       ls=:dash,
    #      )
    # for obs in obstacles
    #     plot!(fig, shape(obs);
    #           seriestype=[:shape,], lw=0.5, c=:blue, linecolor=:black, legend=false, fillalpha=0.2, aspect_ratio=1,
    #          )
    # end
    display(fig)
end


@testset "cbf" begin
    position_cbf()
    position_cbf_full_dynamics()
end
