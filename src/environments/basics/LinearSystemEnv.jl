struct LinearSystemEnv <: AbstractEnv
    n::Int
    m::Int
end

function State(env::LinearSystemEnv)
    @unpack n = env
    return function (x=zeros(n))
        x
    end
end

function Params(env::LinearSystemEnv)
    @unpack n, m = env
    return function (A, B)
        ComponentArray(A=A, B=B)
    end
end

function Dynamics!(env::LinearSystemEnv)
    return function (dx, x, p, t; u)
        @unpack A, B = p
        _u = length(u) == 1 ? u[1] : u
        dx .= A*x + B*_u
        nothing
    end
end
