using JLD2
using FlightSims, OrdinaryDiffEq, ComponentArrays  # for data re-construction
using Parameters: @unpack

using Plots


function _load()
    # run it in a new session. Be careful for loading packages for re-construction.
    saved_data = load("test.jld2")
    @unpack env, prob, sol = saved_data
    df = process(env)(prob, sol)
    plot(df.times, hcat(df.states...)')
end
