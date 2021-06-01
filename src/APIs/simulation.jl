## Simulation
function sim(state0, dyn, p=nothing;
        t0=0.0, tf=1.0, solver=Tsit5(), callback::DiffEqBase.DECallback=CallbackSet(), kwargs...
    )
    tspan = (t0, tf)
    prob = ODEProblem(dyn, state0, tspan, p)
    sol = solve(prob, solver; callback=callback, kwargs...)
    prob, sol
end
"""
    sim(env::AbstractEnv, state0=State(env)(), dyn=dynamics!(env), p=nothing;
    t0=0.0, tf=1.0, solver=Tsit5())
"""
function sim(env::AbstractEnv,
        state0=State(env)(), dyn=dynamics!(env), p=nothing;
        t0=0.0, tf=1.0, solver=Tsit5(), callback::DiffEqBase.DECallback=CallbackSet(), kwargs...
    )
    sim(state0, dyn, p; t0=t0, tf=tf, solver=solver, callback=callback, kwargs...)
end


"""
# Notes
Borrowed from [an MRAC example](https://jonniedie.github.io/ComponentArrays.jl/stable/examples/adaptive_control/).
"""
maybe_apply(f::Function, x, p, t) = f(x, p, t)
maybe_apply(f, x, p, t) = f
function apply_inputs(func; kwargs...)
    simfunc(dx, x, p, t) = func(dx, x, p, t;
                                map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
    # TODO: out-of-place method does not work; see `test/lqr.jl`
    simfunc(x, p, t) = func(x, p, t;
                            map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
    return simfunc
end

