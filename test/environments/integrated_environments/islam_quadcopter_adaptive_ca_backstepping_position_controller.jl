using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test()
    multicopter, controller = FS.IslamQuadcopter_AdaptiveCABacksteppingPositionControllerEnv()
    x0 = State(multicopter, controller)()
    # TODO
    faults = FaultSet(LoE(10.0, 2, 0.9))
    prob, sol = sim(x0, apply_inputs(dynamics!(multicopter, controller; kwargs_multicopter=Dict(:faults => faults)); pos_cmd=[2, 1, 3]);
                    tf=30.0)
    df = process()(prob, sol)
    ts = df.times
    poss = df.states |> Map(state -> state.multicopter.p) |> collect
    adaptive_params = df.states |> Map(state -> state.controller.Θ̂) |> collect
    adaptive_params_flatten = adaptive_params |> Map(Θ̂ -> Θ̂[:]) |> collect
    p_pos = plot(ts, hcat(poss...)')
    savefig("position.png")
    p_adaptive_param = plot(ts, hcat(adaptive_params_flatten...)')
    savefig("adaptive_param.png")
end
