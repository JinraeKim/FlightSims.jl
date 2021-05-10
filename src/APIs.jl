### Default APIs
## Environments
function State(env::AbstractEnv)
    error("Define the structure of state for your env")
end

function dynamics(env::AbstractEnv)
    error("Undefined out-of-place dynamics")
end
function dynamics!(env::AbstractEnv)
    error("Undefined in-place dynamics")
end

## Simulation and data processing
"""
    sim(env::AbstractEnv, state0=State(env)(), dyn=dynamics!(env), p=nothing;
    t0=0.0, tf=1.0, solver=Tsit5())
"""
function sim(env::AbstractEnv,
        state0=State(env)(), dyn=dynamics!(env), p=nothing;
        t0=0.0, tf=1.0, solver=Tsit5(),
    )
    tspan = (t0, tf)
    prob = ODEProblem(dyn, state0, tspan, p)
    sol = solve(prob, solver)
    prob, sol
end

function process(env::AbstractEnv)
    return function (prob::ODEProblem, sol::ODESolution; Δt=0.01)
        t0, tf = prob.tspan
        ts = t0:Δt:tf
        xs = ts |> Map(t -> sol(t)) |> collect
        DataFrame(times=ts, states=xs)
    end
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
