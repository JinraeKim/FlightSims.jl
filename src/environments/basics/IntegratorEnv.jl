struct IntegratorEnv <: AbstractEnv
end

function State(env::IntegratorEnv)
    return function (x::AbstractArray)
        x
    end
end

function dynamics!(env::IntegratorEnv)
    return function (dx, x, p, t; u)
        dx .= u
        nothing
    end
end
