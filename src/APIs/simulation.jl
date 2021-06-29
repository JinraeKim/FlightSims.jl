"""
Extended to deal with empty Dict.
"""
function _namedtuple(x::Dict)
    if x == Dict()
        return NamedTuple()  # empty NamedTuple
    else
        return NamedTupleTools.namedtuple(x)
    end
end


"""
# Notes
- Currently, only iip (isinplace) method is supported.
# Keyword arguments
# Tips
- progress (kwarg): if true, it shows the progress bar (in Juno IDE or supports by [TerminalLoggers.jl](https://github.com/c42f/TerminalLoggers.jl)).
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
    iip = true
    __dyn = (dx, x, p, t) -> dyn(dx, x, p, t)
    prob = ODEProblem{iip}(__dyn, state0, tspan, p)  # true: isinplace
    saved_values = SavedValues(Float64, Dict)
    cb_save = nothing
    if log_off == false
        # logging function
        if isinplace(prob)
            __log_indicator__ = __LOG_INDICATOR__()  # just an indicator for logging
            if hasmethod(dyn, Tuple{Any, Any, Any, Any, __LOG_INDICATOR__})
                log_func = function (x, t, integrator::DiffEqBase.DEIntegrator; kwargs...)
                    x = copy(x)  # `x` merely denotes a "view"
                    dyn(zero.(x), x, integrator.p, t, __log_indicator__; kwargs...)
                end
                cb_save = SavingCallback(log_func, saved_values;
                                         saveat=saveat, tdir=Int(sign(tspan[2]-tspan[1])))
            end
        else
            error("Not tested")
        end
        callback = CallbackSet(callback, cb_save)  # save values "after all other callbacks"
    end
    sol = solve(prob, solver;
                callback=callback,
                kwargs...)
    if log_off == true
        return prob, sol
    else
        # recursive NamedTuple conversion from Dict; https://discourse.julialang.org/t/how-to-make-a-named-tuple-from-a-dictionary/10899/34?u=ihany
        recursive_namedtuple(x::Any) = x
        recursive_namedtuple(d::Dict) = _namedtuple(
                                                    Dict(k => recursive_namedtuple(v) for (k, v) in d)
                                                   )
        df = DataFrame(
                       time = saved_values.t,
                       sol = saved_values.saveval |> Map(recursive_namedtuple) |> collect,
                      )
        return prob, df
    end
end


"""
# Notes
- The basic concept is borrowed from [an MRAC example](https://jonniedie.github.io/ComponentArrays.jl/stable/examples/adaptive_control/).
- It is modified to be compatible with [SimulationLogger.jl](https://github.com/JinraeKim/SimulationLogger.jl).
# Limitations
- Conditional method definition is troublesome; see [#16](https://github.com/JinraeKim/FlightSims.jl/issues/16).
Instead of it, I decided to merely define a new method with argument `__log_indicator__::__LOG_INDICATOR__`,
which will provide "empty Dict" in the case of no logging.
"""
maybe_apply(f::Function, x, p, t) = f(x, p, t)
maybe_apply(f, x, p, t) = f
function apply_inputs(func; kwargs...)
    function simfunc(dx, x, p, t)
        func(dx, x, p, t; map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
    end
    function simfunc(dx, x, p, t, __log_indicator__::__LOG_INDICATOR__)
        if hasmethod(func, Tuple{Any, Any, Any, Any, __LOG_INDICATOR__})
            return func(dx, x, p, t, __log_indicator__; map(f -> maybe_apply(f, x, p, t), (; kwargs...))...)
        else
            return Dict()  # see the above method `NamedTupleTools.namedtuple`
        end
    end
    simfunc
end

# function save_inputs(func; kwargs...)
#     datum_format(x, t, integrator) = func(x, t, integrator;
#                                           map(f -> maybe_apply(f, x, integrator.p, t), (; kwargs...))...)
#     datum_format
# end
