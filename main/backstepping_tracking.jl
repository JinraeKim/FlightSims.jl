using FlightSims
const FS = FlightSims
using UnPack, ComponentArrays
using Transducers
using Plots


function make_env()
    multicopter = IslamQuadcopterEnv()
    @unpack m, g, B = multicopter
    mixer = PseudoInverseMixer(multicopter.B)
    x0_multicopter = State(multicopter)()
    pos0 = copy(x0_multicopter.p)
    vel0 = copy(x0_multicopter.v)
    helixCG = FS.HelixCommandGenerator(pos0)
    cg = command_generator(helixCG)
    controller = BacksteppingPositionControllerEnv(m; x_cmd_func=cg)
    x0_controller = State(controller)(pos0, m, g)
    x0 = ComponentArray(multicopter=x0_multicopter, controller=x0_controller)
    multicopter, controller, mixer, x0, cg
end

function main()
    multicopter, controller, mixer, x0, cg = make_env()
    prob, sol = sim(x0, Dynamics!(multicopter, controller, mixer); tf=40.0)
    t0, tf = prob.tspan
    Δt = 0.01  # data sampling period; not simulation time step
    ts = t0:Δt:tf
    poss = ts |> Map(t -> sol(t).multicopter.p) |> collect
    poss_ref = ts |> Map(t -> cg(t)) |> collect
    ## plot
    # 3d traj
    p_traj = plot3d(; title="position", legend=:outertopright)
    plot!(p_traj, hcat(poss...)'[:, 1], hcat(poss...)'[:, 2], hcat(poss...)'[:, 3]; label="position", color="red")
    plot!(p_traj, hcat(poss_ref...)'[:, 1], hcat(poss_ref...)'[:, 2], hcat(poss_ref...)'[:, 3]; label="position (ref)", color="black")
    savefig(p_traj, "figures/traj_multicopter_backstepping.png")
end
