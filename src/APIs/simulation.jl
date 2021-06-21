"""
# Notes
"""
function sim(state0, dyn, p=nothing;
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
    prob = ODEProblem(dyn, state0, tspan, p)
    saved_values = nothing
    if log_off == false
        # logging function
        log_func = nothing
        if isinplace(prob)
            __log__indicator = __LOG_INDICATOR__()  # just an indicator for logging
            log_func = function (x, t, integrator::DiffEqBase.DEIntegrator; kwargs...)
                x = copy(x)  # `x` merely denotes a "view"
                dyn(zero.(x), x, integrator.p, t, __log__indicator; kwargs...)
            end
        else
            error("Not tested")
        end
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
        # recursive NamedTuple conversion from Dict; https://discourse.julialang.org/t/how-to-make-a-named-tuple-from-a-dictionary/10899/34?u=ihany
        recursive_namedtuple(x::Any) = x
        recursive_namedtuple(d::Dict) = NamedTupleTools.namedtuple(Dict(k => recursive_namedtuple(v) for (k, v) in d))
        for key in all_keys
            _getindex(x) = haskey(x, key) ? getindex(x, key) : missing
            df[!, key] = saved_values.saveval |> Map(_getindex) |> Map(recursive_namedtuple) |> collect
        end
        return prob, df
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
    simfunc(dx, x, p, t, __log__indicator::__LOG_INDICATOR__) = func(dx, x, p, t, __log__indicator;
                                map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
    # TODO: out-of-place method does not work; see `test/lqr.jl`
    simfunc(x, p, t) = func(x, p, t;
                            map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
    simfunc(x, p, t, __log__indicator::__LOG_INDICATOR__) = func(x, p, t, __log__indicator;
                                map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
    return simfunc
end

function save_inputs(func; kwargs...)
    datum_format(x, t, integrator) = func(x, t, integrator;
                                          map(f -> maybe_apply(f, x, integrator.p, t), (; kwargs...))...)
    datum_format
end
