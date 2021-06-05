abstract type QuadcopterEnv <: MulticopterEnv end

"""
Common state structure of QuadcopterEnv
"""
function State(env::QuadcopterEnv)
    return function (p=zeros(3), v=zeros(3), R=one(RotMatrix{3}), ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
    end
end

"""
Input saturation
"""
function saturate_func(env::QuadcopterEnv)
    error("Not implemented")
end

"""
Common dynamics of QuadcopterEnv
"""
function _dynamics!(env::QuadcopterEnv)
    @unpack m, g, J = env
    J_inv = inv(J)
    e3 = [0, 0, 1]
    skew(x) = [    0 -x[3]  x[2];
                x[3]     0 -x[1];
               -x[2]  x[1]    0]
    return function (dX, X, p, t; f, M)
        @unpack p, v, R, ω = X
        Ω = skew(ω)
        dX.p = v
        dX.v = -(1/m)*f*R'*e3 + g*e3
        dX.R = -Ω*R
        dX.ω = J_inv * (-Ω*J*ω + M)
        nothing
    end
end

include("GoodarziQuadcopterEnv.jl")
include("IslamQuadcopterEnv.jl")
