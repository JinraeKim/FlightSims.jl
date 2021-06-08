using FlightSims
const FS = FlightSims


function test()
    env = LeeHexacopterEnv()
    x0 = State(env)()
    prob, sol = sim(x0, apply_inputs(Dynamics!(env); u=ones(6)); tf=10.0)
    df = Process(env)(prob, sol)
end
