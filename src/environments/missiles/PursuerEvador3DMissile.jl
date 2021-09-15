struct PursuerEvador3DMissile <: AbstractMissile
    pursuer::PointMass3DMissile
    evador::PointMass3DMissile
end

function State(env::PursuerEvador3DMissile)
    @unpack pursuer, evador = env
    return function (x0_pursuer, x0_evador)
        ComponentArray(pursuer=x0_pursuer, evador=x0_evador)
    end
end

function Dynamics!(env::PursuerEvador3DMissile)
    @unpack pursuer, evador = env
    @Loggable function dynamics!(dx, x, params, t; u_pursuer, u_evador)
        @onlylog r = norm(x.pursuer.p - x.evador.p)
        @nested_log :pursuer Dynamics!(pursuer)(dx.pursuer, x.pursuer, nothing, t; u=u_pursuer)
        @nested_log :evador Dynamics!(evador)(dx.evador, x.evador, nothing, t; u=u_evador)
    end
end
