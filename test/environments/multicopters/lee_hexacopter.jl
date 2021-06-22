using FlightSims
const FS = FlightSims


function test()
    env = LeeHexacopterEnv()
    x0 = State(env)()
    prob, df = sim(
                   x0,
                   apply_inputs(FS.__Dynamics!(env); f=0.0, M=zeros(3));
                   tf=10.0,
                  )
    df
end
