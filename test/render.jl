using FlightSims
const FS = FlightSims
using Plots
ENV["GKSwstype"] = "nul"
using LinearAlgebra
using Transducers
using DifferentialEquations
using UnPack


function gen_gif()
    multicopter = LeeHexacopterEnv()
    @unpack m, g = multicopter
    x0 = State(multicopter)()
    anim = Animation()
    function affect!(integrator)
        state = copy(integrator.u)
        fig = plot(multicopter, state;
              xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 20),
              camera=(45, 45),
             )
        frame(anim)
    end
    Δt = 0.1
    tf = 10.0
    cb = PeriodicCallback(affect!, Δt)
    function Dynamics!(multicopter::MulticopterEnv)
        FS.__Dynamics!(multicopter)
    end
    prob, df = sim(x0,
                   apply_inputs(Dynamics!(multicopter);
                                f=(state, p, t) -> m*g + (0.5*tf-t),
                                M=zeros(3),
                               );
                   tf=tf,
                   callback=cb)
    gif(anim, "figures/anim.gif", fps=60)
    nothing
end

function topview()
    multicopter = LeeHexacopterEnv()
    x0 = State(multicopter)()
    fig = plot(multicopter, x0;
               ticks=nothing, border=:none,
               # background_color=:transparent,
               xlabel="", ylabel="", zlabel="",
               camera=(0, 90),
               dpi=300,
              )
    savefig(fig, "topview.pdf")
end

function model_description()
    multicopter = LeeHexacopterEnv()
    x0 = State(multicopter)()
    fig = plot(multicopter, x0)
    plot!(fig, )
end

function test()
    gen_gif()
    topview()
    # model_description()  # TODO
end
