using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test()
    m = 1.0
    pos0 = zeros(3)
    g = 9.81
    controller = AdaptiveCABacksteppingPositionControllerEnv(m)
    x0_controller = State(controller)(pos0, m, g)
    p_controller = Params(controller)()
    prob, sol = sim(x0_controller,
                    apply_inputs(dynamics!(controller); pos_cmd=[2, 1, 3], Ṫd=1.0, Θ̂̇=ones(6, 4)),
                    p_controller;
                    tf=10.0)
    df = process(controller)(prob, sol; Δt=0.01)
end
