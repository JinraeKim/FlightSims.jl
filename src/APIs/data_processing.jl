function process()
    return function (prob::ODEProblem, sol::ODESolution; Δt=0.01)
        t0, tf = prob.tspan
        ts = t0:Δt:tf
        xs = ts |> Map(t -> sol(t)) |> collect
        DataFrame(times=ts, states=xs)
    end
end
# will be deprecated
function process(env::AbstractEnv)
    process()
end


# save and load
"""
    save(path::String,
    env::AbstractEnv, prob::ODEProblem, sol::ODESolution;
    process=nothing)
"""
function FileIO.save(path::String,
        env::AbstractEnv, prob::ODEProblem, sol::ODESolution;
        process=nothing,)
    will_be_saved = Dict("env" => env, "prob" => prob, "sol" => sol)
    if process != nothing
        will_be_saved["process"] = process
    end
    FileIO.save(path, will_be_saved)
end

"""
    load(path::String; with_process=false)
"""
function JLD2.load(path::String; with_process=false)
    strs = [:env, :prob, :sol]
    if with_process
        push!(strs, :process)
    end
    saved_data = FileIO.load(path, String.(strs)...)
    (; (zip(strs, saved_data) |> Dict)...)  # NamedTuple
end
