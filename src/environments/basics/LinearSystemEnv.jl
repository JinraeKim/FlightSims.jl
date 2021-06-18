struct LinearSystemEnv <: AbstractEnv
    A
    B
end

function State(env::LinearSystemEnv)
    @unpack B = env
    n = size(B)[1]
    return function (x)
        @assert length(x) == n
        x
    end
end

function Params(env::LinearSystemEnv)
    () -> nothing
end

function Dynamics!(env::LinearSystemEnv)
    @unpack A, B = env
    function dynamics!(dx, x, p, t; u)
        _u = length(u) == 1 ? u[1] : u
        dx .= A*x + B*_u
        nothing
    end
end
