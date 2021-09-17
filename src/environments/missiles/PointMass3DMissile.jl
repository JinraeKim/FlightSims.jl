struct PointMass3DMissile <: AbstractMissile
end

function State(env::PointMass3DMissile)
    return function (p=zeros(3), v=zeros(3))
        ComponentArray(p=p, v=v)
    end
end


"""
u ∈ ℝ³: acceleration
"""
function Dynamics!(env::PointMass3DMissile)
    @Loggable function dynamics!(dx, x, params, t; u)
        @unpack p, v = x
        @log p
        @log v
        dx.p = v
        dx.v = u
    end
end
