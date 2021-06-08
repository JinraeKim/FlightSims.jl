struct LinearSystemEnv <: AbstractEnv
    A
    B
    Q
    R
end

function State(env::LinearSystemEnv)
    @unpack B = env
    n = size(B)[1]
    return function (x=zeros(n))
        x
    end
end

function dynamics!(env::LinearSystemEnv)
    @unpack A, B = env
    return function (dx, x, p, t; u)
        _u = length(u) == 1 ? u[1] : u
        dx .= A*x + B*_u
        nothing
    end
end
