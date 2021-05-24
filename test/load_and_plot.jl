using JLD2
using FlightSims, OrdinaryDiffEq, ComponentArrays  # for data re-construction
using UnPack

using Plots


function _load()
    # run it in a new session. Be careful for loading packages for re-construction.
    # It would fail to serialise anonymous functions by JLD2. See https://github.com/JuliaIO/JLD2.jl/issues/314.
    dir_log = "data/sim_and_save"
    saved_data = load(joinpath(dir_log, "test.jld2"))
    @unpack env, prob, sol = saved_data
    df = process(env)(prob, sol)
    plot(df.times, hcat(df.states...)')
end
