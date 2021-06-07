using FlightSims
const FS = FlightSims
using UnPack, ComponentArrays
using Transducers
using Plots


function make_env()
    multicopter = IslamQuadcopterEnv()
    @unpack m, g = multicopter
    x0_multicopter = State(multicopter)()
    pos0 = copy(x0_multicopter.p)
    helixCG = FS.HelixCommandGenerator(pos0)
    cg = command_generator(helixCG)
    controller = BacksteppingPositionControllerEnv(m; x_cmd_func=cg)
    x0_controller = State(controller)(pos0, m, g)
    x0 = ComponentArray(multicopter=x0_multicopter, controller=x0_controller)
    multicopter, controller, x0, cg
end

function main()
    multicopter, controller, x0, cg = make_env()
    prob, sol = sim(x0, dynamics!(multicopter, controller); tf=40.0)
    df = process()(prob, sol; Δt=0.01)
    ts = df.times
    poss = df.states |> Map(state -> state.multicopter.p) |> collect
    poss_ref = ts |> Map(t -> cg(t)) |> collect
    ## plot
    # time vs position
    p_pos = plot(; title="position", legend=:outertopright)
    plot!(p_pos, ts, hcat(poss...)'; label=["x" "y" "z"], color="red", ls=[:dash :dot :dashdot])
    plot!(p_pos, ts, hcat(poss_ref...)'; label=["x (ref)" "y (ref)" "z (ref)"], color="black", ls=[:dash :dot :dashdot])
    savefig(p_pos, "t_vs_pos.png")
    # 3d traj
    p_traj = plot3d(; title="position", legend=:outertopright)
    plot!(p_traj, hcat(poss...)'[:, 1], hcat(poss...)'[:, 2], hcat(poss...)'[:, 3]; label="position", color="red")
    plot!(p_traj, hcat(poss_ref...)'[:, 1], hcat(poss_ref...)'[:, 2], hcat(poss_ref...)'[:, 3]; label="position (ref)", color="black")
    savefig(p_traj, "traj.png")
end
