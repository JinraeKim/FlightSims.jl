using FlightSims
using Plots
using JLD2, FileIO


function main()
    dir_log = "data/sim_and_save"
    mkpath(dir_log)
    env = TwoDimensionalNonlinearPolynomialEnv()
    x0 = State(env)()
    # simulation and data processing
    prob, sol = sim(x0, apply_inputs(dynamics!(env); u=FlightSims.optimal_input(env));
                    tf=10.0)
    # save (for stable data saving, one may save raw data). Here, raw data means e.g. t = 0, 0.01, 0.02, ...
    FlightSims.save(joinpath(dir_log, "test.jld2"), env, prob, sol)
    # to load it, see `main/example_load.jl`
end
