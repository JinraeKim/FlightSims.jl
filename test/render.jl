using FlightSims
const FS = FlightSims
using Plots
ENV["GKSwstype"] = "nul"
using LinearAlgebra
using Transducers
using DifferentialEquations
using UnPack


function test()
    multicopter = LeeHexacopterEnv()
    @unpack m, g = multicopter
    x0 = State(multicopter)()
    anim = Animation()
    function affect!(integrator)
        state = copy(integrator.u)
        fig = plot(multicopter, state;
              xlim=(-1, 1), ylim=(-1, 1), zlim=(-1, 20),
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
