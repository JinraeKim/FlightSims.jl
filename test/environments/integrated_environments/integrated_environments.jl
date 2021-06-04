using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test()
    multicopter, controller = FS.GoodarziQuadcopter_BacksteppingControllerEnv()
    x0 = State(multicopter, controller)()
    pos_cmd = [1, 2, 3]
    prob, sol = sim(x0, apply_inputs(dynamics!(multicopter, controller); pos_cmd=pos_cmd);
                    tf=10.0)
    df = process(controller)(prob, sol; Δt=0.01)
    ts = df.times
    poss = df.states |> Map(state -> state.multicopter.p) |> collect
    pos_cmds = poss |> Map(pos -> pos_cmd) |> collect
    # figures
    p = plot(title="position")
    plot!(p, ts, hcat(poss...)'; label=["x" "y" "z"], color="blue", ls=[:dash :dot :dashdot])
    plot!(p, ts, hcat(pos_cmds...)'; label=["x (cmd)" "y (cmd)" "z (cmd)"], color="black", ls=[:dash :dot :dashdot])
end
