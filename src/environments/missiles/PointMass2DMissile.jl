struct PointMass2DMissile <: AbstractMissile
end

function State(env::PointMass2DMissile)
    return function (p=zeros(2), v=zeros(2))
        ComponentArray(p=p, v=v)
    end
end


"""
u ∈ ℝ^2: acceleration
"""
function Dynamics!(env::PointMass2DMissile)
    @Loggable function dynamics!(dx, x, params, t; u)
        @unpack p, v = x
        @log p
        @log v
        dx.p = v
        dx.v = u
    end
end
