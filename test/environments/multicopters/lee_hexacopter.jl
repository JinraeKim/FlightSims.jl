using FlightSims
const FS = FlightSims


function test()
    env = LeeHexacopterEnv()
    x0 = State(env)()
    prob, sol = sim(x0, apply_inputs(dynamics!(env); u=ones(6)); tf=10.0)
    df = process(env)(prob, sol)
end
