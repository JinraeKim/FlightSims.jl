using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test()
    env = ReferenceModelEnv(4)
    x0 = zeros(3)
    X0_ref = State(env)(x0)
    prob, sol = sim(X0_ref, apply_inputs(dynamics!(env); x_cmd=[1, 2, 3]); tf=10.0)
    df = process(env)(prob, sol)
    xs = df.states |> Map(X -> X.x_0) |> collect
    plot(df.times, hcat(xs...)')
end
