using FlightSims
const FS = FlightSims
using Transducers
using Plots
using ComponentArrays
using UnPack


function test_controller()
    m = 1.0
    pos0 = zeros(3)
    g = 9.81
    controller = BacksteppingPositionController(m)
    x0_controller = State(controller)(pos0, m, g)
    prob, sol = sim(x0_controller,
                    apply_inputs(dynamics!(controller),
                                 pos_cmd=[1, 2, 3], Ṫd=1.0);
                    tf=10.0)
    df = process(controller)(prob, sol; Δt=0.01)
    ts = df.times
    xds = df.states |> Map(state -> state.xd) |> collect
    # Tds = df.states |> Map(state -> state.Td) |> collect
    plot(ts, hcat(xds...)')
    # plot(ts, hcat(Tds...)')
end

function test()
    multicopter = GoodarziQuadcopterEnv()
    @unpack m, g = multicopter
    x0_multicopter = State(multicopter)()
    controller = BacksteppingPositionController(m)
    x0_controller = State(controller)(x0_multicopter.p, m, g)
    x0 = ComponentArray(multicopter=x0_multicopter, controller=x0_controller)
    function feedback_dynamics!(multicopter::GoodarziQuadcopterEnv, controller::BacksteppingPositionController)
        @unpack m, J, g = multicopter
        return function (dx, x, p, t; pos_cmd)
            @unpack p, v, R, ω = x.multicopter
            @unpack xd, vd, ad, ȧd, äd, Td = x.controller
            νd, Ṫd = command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
            dynamics!(controller)(dx.controller, x.controller, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
            dynamics!(multicopter)(dx.multicopter, x.multicopter, (), t; f=νd.f, M=νd.M)
            nothing
        end
    end
    pos_cmd = [1, 2, 3]
    prob, sol = sim(x0, apply_inputs(feedback_dynamics!(multicopter, controller); pos_cmd=pos_cmd);
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
