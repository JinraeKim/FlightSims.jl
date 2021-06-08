using FlightSims
const FS = FlightSims


function test()
    env = IslamQuadcopterEnv()
    x0 = State(env)()
    # faults = AbstractFault[]
    faults = FaultSet(LoE(0.0, 1, 0.1), LoE(0.1, 1, 0.2), LoE(0.2, 2, 0.5))
    prob, sol = sim(x0, apply_inputs(Dynamics!(env; faults=faults); u=ones(4)); tf=10.0)
    df = Process(env)(prob, sol)
end
