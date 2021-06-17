## Simulation
# """
# # Notes
# - `datum_format` should be a function
#     - whose input is `(x, t, integrator::DiffEqBase.DEIntegrator)`
#     and whose output is `nt::NamedTuple`.
#     - See `src/APIs/data_processing.jl`.
# - Default settings for saving
#     - save values "after all other callbacks"
#     - `saveat=[]` will save data at every time step.
# """
# function sim(state0, dyn, p=nothing;
#         t0=0.0, tf=1.0, solver=nothing,  # DifferentialEquations.jl will find a default solver
#         callback::DiffEqBase.DECallback=CallbackSet(),
#         datum_format=nothing,
#         saveat=nothing,
#         savestep=nothing,
#         kwargs...,
#     )
#     if saveat == nothing && savestep == nothing
#         saveat = []  # default
#     elseif saveat != nothing && savestep == nothing
#         # nothing
#     elseif saveat == nothing && savestep != nothing
#         saveat = t0:savestep:tf
#     elseif saveat != nothing && savestep != nothing
#         error("Assign values of either `saveat` or `savestep`")
#     end
#     tspan = (t0, tf)
#     prob = ODEProblem(dyn, state0, tspan, p)
#     saved_values = nothing
#     if datum_format != nothing
#         saved_values = SavedValues(Float64, NamedTuple)
#         cb_save = SavingCallback(datum_format, saved_values;
#                                  saveat=saveat, tdir=Int(sign(tspan[2]-tspan[1])))
#         callback = CallbackSet(callback, cb_save)  # save values "after all other callbacks"
#     end
#     sol = solve(prob, solver; callback=callback, kwargs...)
#     if datum_format == nothing
#         return prob, sol
#     else
#         df = DataFrame(time=saved_values.t)
#         all_keys = (saved_values.saveval |> Map(keys) |> union)[1]
#         for key in all_keys
#             _getproperty(x) = isdefined(x, key) ? getproperty(x, key) : missing
#             df[!, key] = saved_values.saveval |> Map(_getproperty) |> collect
#         end
#         return prob, sol, df
#     end
# end
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
    function dyn__LOG__ end  # necessary
    if isinplace(prob)
        @LOG function dyn(dx, x, p, t; kwargs...)
        #     A = [0 1;
        #          0 0]
        #     B = [0;
        #          1]
        #     u = -1.0
        #     _u = length(u) == 1 ? u[1] : u
        #     @log input = _u
        #     dx .= A*x + B*_u
        #     nothing
            __dyn(dx, x, p, t; kwargs...)
        end  # @LOG generates logger function whose name is dyn__LOG__(x, t, integrator)
    else
        # @LOG function dyn(x, p, t; kwargs...)
        #     __dyn(x, p, t; kwargs...)
        # end  # @LOG generates logger function whose name is dyn__LOG__(x, t, integrator)
        error("Not tested yet")
    end
    saved_values = nothing
    if log_off == false
        saved_values = SavedValues(Float64, NamedTuple)
        # cb_save = SavingCallback(datum_format, saved_values;
        cb_save = SavingCallback(dyn__LOG__, saved_values;
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

function save_inputs(func; kwargs...)
    datum_format(x, t, integrator) = func(x, t, integrator;
                                          map(f -> maybe_apply(f, x, integrator.p, t), (; kwargs...))...)
    datum_format
end
