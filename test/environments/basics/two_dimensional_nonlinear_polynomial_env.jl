using FlightSims
using Plots
using Test


@testset "TwoDimensionalNonlinearPolynomialEnv" begin
    dir_log = "data/test/environments/basics/TwoDimensionalNonlinearPolynomialEnv"
    mkpath(dir_log)

    env = TwoDimensionalNonlinearPolynomialEnv()
    x0 = State(env)()
    @time prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=FlightSims.optimal_input(env)); tf=10.0)
    p = plot(sol)
    savefig(p, joinpath(dir_log, "x.png"))
    println("Test results are saved in $(dir_log)")
end
