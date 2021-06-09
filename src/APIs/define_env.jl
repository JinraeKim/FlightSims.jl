### Default APIs
## Environments
function State(env::AbstractEnv)
    error("Define the structure of state for your environment")
end

function Params(env::AbstractEnv)
    error("Define the structure of parameters for your environment")
end

function Dynamics!(env::AbstractEnv)
    error("Define in-place dynamics")
end

function Dynamics(env::AbstractEnv)
    @warn "It is recommended to use `Dynamics!`; see https://github.com/JinraeKim/FlightSims.jl/issues/16"
    error("Define out-of-place dynamics")
end
