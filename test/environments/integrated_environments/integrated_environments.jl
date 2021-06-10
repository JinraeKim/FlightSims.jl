using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test_bak()
    multicopter, controller = FS.GoodarziQuadcopter_BacksteppingControllerEnv()
    x0 = State(multicopter, controller)()
    pos_cmd = [1, 2, 3]
    prob, sol = sim(x0, apply_inputs(Dynamics!(multicopter, controller); pos_cmd=pos_cmd);
                    tf=10.0)
    df = Process()(prob, sol; Δt=0.01)
    ts = df.time
    poss = df.state |> Map(state -> state.multicopter.p) |> collect
    pos_cmds = poss |> Map(pos -> pos_cmd) |> collect
    # figures
    p = plot(title="position")
    plot!(p, ts, hcat(poss...)'; label=["x" "y" "z"], color="blue", ls=[:dash :dot :dashdot])
    plot!(p, ts, hcat(pos_cmds...)'; label=["x (cmd)" "y (cmd)" "z (cmd)"], color="black", ls=[:dash :dot :dashdot])
end

function test()
    multicopter, controller = FS.LeeHexacopter_BacksteppingPositionControllerEnv()
    allocator = PseudoInverseControlAllocator(multicopter.B)
    x0 = State(multicopter, controller)()
    pos_cmd = [2, 1, 3]
    prob, sol = sim(x0, apply_inputs(Dynamics!(multicopter, controller, allocator); pos_cmd=pos_cmd);
                    tf=10.0)
    df = Process(multicopter, controller, allocator)(prob, sol)
    p_pos = plot(df.time, hcat(df.position...)')
    savefig(p_pos, "pos.png")
    p_u_cmd = plot(df.time, hcat(df.u_command...)')
    savefig(p_u_cmd, "u_cmd.png")
end
