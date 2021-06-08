## Simulation
"""
# Notes
- `saving_func` should be a function
    - whose input is `(x, t, integrator::DiffEqBase.DEIntegrator)`
    - and whose output is `nt::NamedTuple`.
- Default settings for saving
    - save values "after all other callbacks"
    - `saveat=[]` will save data at every time step.
"""
function sim(state0, dyn, p=nothing;
        t0=0.0, tf=1.0, solver=Tsit5(),
        callback::DiffEqBase.DECallback=CallbackSet(),
        saving_func=nothing, saveat=[],
        kwargs...,
    )
    tspan = (t0, tf)
    prob = ODEProblem(dyn, state0, tspan, p)
    saved_values = nothing
    if saving_func != nothing
        saved_values = SavedValues(Float64, NamedTuple)
        cb_save = SavingCallback(saving_func, saved_values; saveat=saveat, tdir=Int(sign(tspan[2]-tspan[1])))
        callback = CallbackSet(callback, cb_save)  # save values "after all other callbacks"
    end
    sol = solve(prob, solver; callback=callback, kwargs...)
    if saving_func == nothing
        return prob, sol
    else
        df = DataFrame(times=saved_values.t)
        all_keys = (saved_values.saveval |> Map(keys) |> union)[1]
        for key in all_keys
            _getproperty(x) = isdefined(x, key) ? getproperty(x, key) : missing
            df[!, key] = saved_values.saveval |> Map(_getproperty) |> collect
        end
        return prob, sol, df
    end
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

