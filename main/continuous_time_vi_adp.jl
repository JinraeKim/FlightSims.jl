using FlightSims
const FS = FlightSims
using Transducers
using Plots


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

function explore(env, u_explorer)
    x0 = State(env)()
    prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=u_explorer); tf=0.3)
    df = process(env)(prob, sol)
    df.inputs = zip(df.times, df.states) |> MapSplat((t, x) -> u_explorer(x, (), t)) |> collect
    df
end

function train(adp, running_cost)
    @show adp.V̂.w
    lr = 1e-1
    u_norm_max = 3.0
    @run FS.update!(adp, running_cost, lr; u_norm_max=u_norm_max)
    @show adp.V̂.w
end

function main()
    env, u_explorer, adp, n, m = initialise()
    df = explore(env, u_explorer)
    @run FS.set_data!(adp, df)
    train(adp, FS.running_cost(env))
end
