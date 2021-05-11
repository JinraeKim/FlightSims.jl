using FlightSims
const FS = FlightSims
using Transducers
using Plots


function explore(env, u_explorer)
    x0 = State(env)()
    prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=1.8)
    df = process(env)(prob, sol)
    df.inputs = zip(df.times, df.states) |> MapSplat((t, x) -> u_explorer(x, (), t)) |> collect
    df
end

function train(adp, training_data)
    FS.set_data!(adp)(training_data)
    error("TODO")
end

function initialise()
    # setting
    env = TwoDimensionalNonlinearPolynomialEnv()
    ω = 1
    u_explorer = (x, p, t) -> sin(2*π*ω*t)
    __x = State(env)()
    __t = 0.0
    n = length(__x)
    __u = u_explorer(__x, (), __t) 
    m = length(__u)
    adp = CTValueIterationADP(n, m)
    env, u_explorer, adp, n, m
end

function main()
    env, u_explorer, adp, n, m = initialise()
    # explore and train
    df = explore(env, u_explorer)
    train(adp, df)
end
