using FlightSims
using Transducers
using OrdinaryDiffEq


function _get_data(prob::ODEProblem, sol::ODESolution)
    Δt = 0.01
    t0, tf = prob.tspan
    ts = t0:Δt:tf
    xs = ts |> Map(t -> sol(t)) |> collect
end

function test()
    env = TwoDimensionalNonlinearPolynomialEnv()
    df = process(sim(env)...)
end
