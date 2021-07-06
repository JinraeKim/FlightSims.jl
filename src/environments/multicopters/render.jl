"""
    render!(multicopter::LeeHexacopterEnv, fig, state)

fig: figure object, i.e., fig = plot()
state: see `State(env::MulticopterEnv)`.
# Notes
Default configuration: ENU-coordinate system
"""
function plot!(fig::Plots.Plot, multicopter::LeeHexacopterEnv, state;
        xlabel="E (m)", ylabel="N (m)", zlabel="U (m)",
        xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 1),
        kwargs...)
    @unpack l = multicopter
    airframe_ref = airframe_reference(multicopter)
    @unpack p, v, R, ω = state
    p_enu = ned2enu(p)
    if (p_enu[1] < xlim[1] || p_enu[1] > xlim[2] ||
        p_enu[2] < ylim[1] || p_enu[2] > ylim[2] ||
        p_enu[3] < zlim[1] || p_enu[3] > zlim[2])
        @warn "Your multicopter is out of the scope"
    end
    body_u = R'*[0, 0, -1]  # B to I
    # plotting
    plot!(fig;
          xlabel=xlabel, ylabel=ylabel, zlabel=zlabel,
          xlim=xlim, ylim=ylim, zlim=zlim,
          kwargs...)
    plot_fuselage!(fig, p, l)
    plot_rotors!(fig, p, l, l/2.5, body_u, airframe_ref)
    plot_frames!(fig, p, l, airframe_ref)
    fig
end

function plot(multicopter::LeeHexacopterEnv, state; kwargs...)
    fig = plot()
    plot!(fig, multicopter, state; kwargs...)
    fig
end

"""
c: centre
r: radius
h: direction of normal vector
"""
function circle_shape(c, r, h)
    normal_vec1 = nothing
    while true
        rand_vec = rand(3)
        normal_vec1 = cross(h, rand_vec)
        if norm(normal_vec1) > 0
            normal_vec1 = normal_vec1 / norm(normal_vec1)
            break
        end
    end
    normal_vec2 = cross(h, normal_vec1)
    normal_vec2 = normal_vec2 / norm(normal_vec2)
    θs = LinRange(0, 2*π-1e-2, 500)
    circle = θs |> Map(θ -> c + r*cos(θ)*normal_vec1 + r*sin(θ)*normal_vec2) |> collect
    circle_1 = circle |> Map(p -> p[1]) |> collect
    circle_2 = circle |> Map(p -> p[2]) |> collect
    circle_3 = circle |> Map(p -> p[3]) |> collect
    circle_1, circle_2, circle_3
end

function plot_fuselage!(fig::Plots.Plot, p, l)
    p_enu = ned2enu(p)
    point_1 = p + 0.2*l*[1, 0, 0]
    point_2 = p + 0.2*l*[0, 1, 0]
    point_3 = p + 0.2*l*[-1, 0, 0]
    point_4 = p + 0.2*l*[0, -1, 0]
    boundary_12_enu = ned2enu.(LinRange(point_1, point_2, 100))
    boundary_23_enu = ned2enu.(LinRange(point_2, point_3, 100))
    boundary_34_enu = ned2enu.(LinRange(point_3, point_4, 100))
    boundary_41_enu = ned2enu.(LinRange(point_4, point_1, 100))
    plot!(fig,
          [_p[1] for _p in boundary_12_enu],
          [_p[2] for _p in boundary_12_enu],
          [_p[3] for _p in boundary_12_enu],
          c=:red, label=nothing)  # position
    plot!(fig,
          [_p[1] for _p in boundary_23_enu],
          [_p[2] for _p in boundary_23_enu],
          [_p[3] for _p in boundary_23_enu],
          c=:red, label=nothing)  # position
    plot!(fig,
          [_p[1] for _p in boundary_34_enu],
          [_p[2] for _p in boundary_34_enu],
          [_p[3] for _p in boundary_34_enu],
          c=:red, label=nothing)  # position
    plot!(fig,
          [_p[1] for _p in boundary_41_enu],
          [_p[2] for _p in boundary_41_enu],
          [_p[3] for _p in boundary_41_enu],
          c=:red, label=nothing)  # position
end

function rotor_positions(p0, l, airframe_ref)
    if airframe_ref == :hexa_x
        pf_1 = p0 + l*[0, 1, 0]
        pf_2 = p0 + l*[0, -1, 0]
        pf_3 = p0 + l*[cos(deg2rad(-30)), sin(deg2rad(-30)), 0]
        pf_4 = p0 + l*[cos(deg2rad(150)), sin(deg2rad(150)), 0]
        pf_5 = p0 + l*[cos(deg2rad(30)), sin(deg2rad(30)), 0]
        pf_6 = p0 + l*[cos(deg2rad(210)), sin(deg2rad(210)), 0]
        return [pf_1, pf_2, pf_3, pf_4, pf_5, pf_6]
    else
        error("Invalid airframe reference")
    end
end

function plot_rotor!(fig::Plots.Plot, p, r, body_u, color)
    body_u_enu = ned2enu(body_u)
    p_enu = ned2enu(p)
    plot!(fig,
          circle_shape(p_enu, r, body_u_enu);
          c=color, opacity=0.25, lw=1.5, label=nothing,
         )
end

function plot_rotors!(fig::Plots.Plot, p0, l, r, body_u, airframe_ref)
    if airframe_ref == :hexa_x
        pf_1, pf_2, pf_3, pf_4, pf_5, pf_6 = rotor_positions(p0, l, airframe_ref)
        plot_rotor!(fig, pf_1, r, body_u, :green)
        plot_rotor!(fig, pf_2, r, body_u, :blue)
        plot_rotor!(fig, pf_3, r, body_u, :green)
        plot_rotor!(fig, pf_4, r, body_u, :blue)
        plot_rotor!(fig, pf_5, r, body_u, :blue)
        plot_rotor!(fig, pf_6, r, body_u, :green)
    else
        error("Invalid airframe reference")
    end
end

function plot_frame!(fig::Plots.Plot, p0, pf)
    frame = LinRange(p0, pf, 100)
    frame_enu = ned2enu.(frame)
    plot!(fig,
          [_p[1] for _p in frame_enu],
          [_p[2] for _p in frame_enu],
          [_p[3] for _p in frame_enu],
          c=:black, opacity=0.50, lw=2.0, label=nothing,
         )
end

function plot_frames!(fig::Plots.Plot, p0, l, airframe_ref)
    p0_enu = ned2enu(p0)
    if airframe_ref == :hexa_x
        pf_1, pf_2, pf_3, pf_4, pf_5, pf_6 = rotor_positions(p0, l, airframe_ref)
        plot_frame!(fig, p0, pf_1)  # 1
        plot_frame!(fig, p0, pf_2)  # 2
        plot_frame!(fig, p0, pf_3)  # 3
        plot_frame!(fig, p0, pf_4)  # 4
        plot_frame!(fig, p0, pf_5)  # 5
        plot_frame!(fig, p0, pf_6)  # 6
    else
        error("Invalid airframe reference")
    end
end
