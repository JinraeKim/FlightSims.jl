### Default APIs
## Environments
function State(env::AbstractEnv)
    error("Define the structure of state for your env")
end

function Params(env::AbstractEnv)
    error("Define the structure of parameters for your env")
end

function dynamics(env::AbstractEnv)
    error("Undefined out-of-place dynamics")
end

function dynamics!(env::AbstractEnv)
    error("Undefined in-place dynamics")
end
