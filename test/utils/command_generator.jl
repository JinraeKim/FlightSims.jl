using FlightSims
const FS = FlightSims
using UnPack
using Transducers
using Plots


function test()
    power_loop = PowerLoop()
    cg = command_generator(power_loop)
    @unpack t0, t_go_straight, t_loop = power_loop
    Δt = 0.01
    tf = t0 + 2*t_go_straight + t_loop + 5
    ts = t0:Δt:tf
    ps = ts |> Map(cg) |> collect
    ps_cat = hcat(ps...)'
    p_traj = plot3d(1;
                    xlim=(-20, 20),
                    ylim=(-20, 20),
                    zlim=(-20, 20),
                    aspect_ratio=:equal,
                    label="trajectory",
                    title="Power Loop")
    anim = Animation()
    for p in ps[1:10:end]
        push!(p_traj, p...)
        frame(anim)
    end
    gif(anim, "power_loop.gif")
end
