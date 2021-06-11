struct IntegratorEnv <: AbstractEnv
end

"""
Even for scalar integrator, you should put an array of length 1;
due to the limitation of in-place method `Dynamics!`.
"""
function State(env::IntegratorEnv)
    return function (x::AbstractArray)
        x
    end
end

function Dynamics!(env::IntegratorEnv)
    return function (dx, x, p, t; u)
        @assert length(dx) == length(u)
        dx .= u
        nothing
    end
end
