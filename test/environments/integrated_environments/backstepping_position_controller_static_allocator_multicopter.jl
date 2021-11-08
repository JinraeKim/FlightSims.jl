using FlightSims
const FS = FlightSims

using Plots
using Transducers
using Test

function get_traj_data(env, Δt, tf)
    x0 = State(env)()
    simulator = Simulator(x0, Dynamics!(env);
                          tf=tf)
    df = solve(simulator; savestep=Δt)
    df
end

function plot_figures!(fig, df, pos_cmd_func)
    ts = df.time
    poss = df.sol |> Map(datum -> datum.multicopter.state.p) |> collect
    poss_cmd = ts |> Map(pos_cmd_func) |> collect
    plot!(fig,
          ts, hcat(poss_cmd...)';
          label="desired",
          color=:red,
         )
    plot!(fig,
          ts, hcat(poss...)';
          label="true",
          color=:blue,
         )
end

function main()
    pos_cmd_func = (t) -> [sin(t), cos(t), t]
    env = FS.BacksteppingPositionController_StaticAllocator_Multicopter(pos_cmd_func)
    Δt = 0.01
    tf = 10.0
    df = get_traj_data(env, Δt, tf)
    # plot
    fig = plot(; legend=:topleft)
    plot_figures!(fig, df, pos_cmd_func)
    savefig("figures/multicopter_position_control.png")
    display(fig)
end

@testset "hexacopter position control example" begin
    main()
end
