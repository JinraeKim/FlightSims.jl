"""
LinearSystemEnv + IntegratorEnv
"""
function LinearSystem_IntegratorEnv(args_linearsystem)
    linearsystem = LinearSystemEnv(args_linearsystem...)
    integ = IntegratorEnv()
    linearsystem, integ
end

"""
# Notes
- `r` denotes integrand of `integ`.
"""
function State(linearsystem::LinearSystemEnv, integ::IntegratorEnv)
    return function (x0_linearsystem, x0_integrator=[0.0])
        x_linearsystem = State(linearsystem)(x0_linearsystem)
        x_integrator = State(integ)(x0_integrator)
        ComponentArray(x=x_linearsystem, ∫r=x_integrator)
    end
end

"""
# Notes
running_cost: integrand; function of (x, u)
"""
function Dynamics!(linearsystem::LinearSystemEnv, integ::IntegratorEnv, running_cost::Function)
    return function (dX, X, p, t; u)
        @unpack x = X
        Dynamics!(linearsystem)(dX.x, X.x, (), t; u=u)
        r = running_cost(x, u)
        Dynamics!(integ)(dX.∫r, X.∫r, (), t; u=r)
        nothing
    end
end
