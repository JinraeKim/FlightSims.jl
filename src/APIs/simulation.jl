"""
# Notes
"""
function sim(state0, __dyn, p=nothing;
        t0=0.0, tf=1.0, solver=nothing,  # DifferentialEquations.jl will find a default solver
        callback::DiffEqBase.DECallback=CallbackSet(),
        log_off=false,
        saveat=nothing,
        savestep=nothing,
        kwargs...,
    )
    if saveat == nothing && savestep == nothing
        saveat = []  # default
    elseif saveat != nothing && savestep == nothing
        # nothing
    elseif saveat == nothing && savestep != nothing
        saveat = t0:savestep:tf
    elseif saveat != nothing && savestep != nothing
        error("Assign values of either `saveat` or `savestep`")
    end
    tspan = (t0, tf)
    prob = ODEProblem(__dyn, state0, tspan, p)
    log_func = nothing
    if isinplace(prob)
        log_func = (x, t, integrator::DiffEqBase.DEIntegrator; kwargs...) -> __dyn(zero.(x), x, integrator.p, t; kwargs...)
    else
        error("Not tested")
    end
    saved_values = nothing
    if log_off == false
        saved_values = SavedValues(Float64, Dict)
        cb_save = SavingCallback(log_func, saved_values;
                                 saveat=saveat, tdir=Int(sign(tspan[2]-tspan[1])))
        callback = CallbackSet(callback, cb_save)  # save values "after all other callbacks"
    end
    sol = solve(prob, solver; callback=callback, kwargs...)
    if log_off == true
        return prob, sol
    else
        df = DataFrame(time=saved_values.t)
        all_keys = (saved_values.saveval |> Map(keys) |> union)[1]
        for key in all_keys
            _getindex(x) = haskey(x, key) ? getindex(x, key) : missing
            df[!, key] = saved_values.saveval |> Map(_getindex) |> collect
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

function save_inputs(func; kwargs...)
    datum_format(x, t, integrator) = func(x, t, integrator;
                                          map(f -> maybe_apply(f, x, integrator.p, t), (; kwargs...))...)
    datum_format
end
