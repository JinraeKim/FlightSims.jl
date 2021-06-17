macro LOG(defun)
    _def = splitdef(defun)  # original
    def = copy(_def)
    def[:name] = Symbol(String(_def[:name]) * String(:__LOG__))
    body = _def[:body]
    args = _def[:args]
    x_expr = args[end-2]  # dx, x, p, t or x, p, t -> x
    p_expr = args[end-1]  # dx, x, p, t or x, p, t -> p
    t_expr = args[end]  # dx, x, p, t or x, p, t -> t
    def[:args] = (x_expr, t_expr, :integrator)
    def[:body] = quote
        __LOGGER_DICT__ = @isdefined($(:__LOGGER_DICT__)) ? __LOGGER_DICT__ : Dict()
        # __LOGGER_DICT__ = Dict()
        # copy for view issue: https://diffeq.sciml.ai/stable/features/callback_library/#Constructor-5
        $(args[end-2]) = copy($(def[:args][1]))  # dx, x, p, t or x, p, t -> x
        if length($args) == 4
            $(args[end-3]) = zero.($(args[end-2]))  # dx; zero-initialisation
        end
        $(args[end-1]) = getfield($(def[:args][3]), :p) == nothing ? getfield($(def[:args][3]), :p) : copy(getfield($(def[:args][3]), :p))  # dx, x, p, t or x, p, t -> p
        # $(args[end-1]) = getfield($(def[:args][3]), :p)  # dx, x, p, t or x, p, t -> p
        $(args[end]) = copy($(def[:args][2]))  # dx, x, p, t or x, p, t -> t
        # __LOGGER_DICT__[:state] = $(args[end-2])  # default saving data: state
        # __LOGGER_DICT__[:time] = $(args[end])  # default saving data: time
        $(body.args[1:end-1]...)  # remove the last line, return, to return __LOGGER_DICT__ 
        return __LOGGER_DICT__  # Dictionary (see `sim`; just a convention)
    end
    res = quote
        $(MacroTools.combinedef(_def))
        $(MacroTools.combinedef(def))
    end
    esc(res)
end

macro log(expr)
    esc(:(@isdefined($:__LOGGER_DICT__) ? @log($:__LOGGER_DICT__, $expr) : $expr))
end
macro log_only(expr)
    esc(:(@isdefined($:__LOGGER_DICT__) ? @log($:__LOGGER_DICT__, $expr) : nothing))
end

macro log(__LOGGER_DICT__, expr)
    return if expr isa Symbol
        quote
            local val = $(esc(expr))
            local var_name = $((expr,))[1]
            local logger_dict = $(esc(__LOGGER_DICT__))
            # if !haskey(logger_dict, var_name)
            #     logger_dict[var_name] = typeof(val)[]
            # end
            # push!(getindex(logger_dict, var_name), val)
            haskey(logger_dict, var_name) ? error("Already defined key: $(var_name)") : setindex!(logger_dict, val, var_name)
            nothing
        end
    elseif expr.head == :(=)
        if expr.args[1] isa Expr && expr.args[1].head == :tuple
            quote
                local vals = $(esc(expr.args[2]))
                local var_names = $(esc(expr.args[1].args))
                local logger_dict = $(esc(__LOGGER_DICT__))
                for (var_name, val) in zip(var_names, vals)
                    # if !haskey(logger_dict, var_name)
                    #     logger_dict[var_name] = typeof(val)[]
                    # end
                    # push!(getindex(logger_dict, var_name), val)
                    # logger_dict[var_name] = val
                    haskey(logger_dict, var_name) ? error("Already defined key: $(var_name)") : setindex!(logger_dict, val, var_name)
                end
                $(esc(expr))
            end
        else
            quote
                local val = $(esc(expr.args[2]))
                local var_name = $((expr.args[1],))[1]
                local logger_dict = $(esc(__LOGGER_DICT__))
                # if !haskey(logger_dict, var_name)
                #     logger_dict[var_name] = typeof(val)[]
                # end
                # push!(getindex(logger_dict, var_name), val)
                # logger_dict[var_name] = val
                haskey(logger_dict, var_name) ? error("Already defined key: $(var_name)") : setindex!(logger_dict, val, var_name)
                $(esc(expr))
            end
        end
    else
        :(error("To log a variable, use either one of forms: `@log val` or `@log var_name = val`"))
    end
end

macro nested_log(symbol, expr)
    if expr.head == :call
        _expr = copy(expr)
        expr.args[1] = Symbol(String(_expr.args[1]) * String(:__LOG__))  # function name (e.g., my_func -> my_func__LOG__)
        if length(expr.args) >= 5 && typeof(expr.args[end-3]) == Symbol
            expr.args = [expr.args[1:end-4]..., expr.args[end-2:end]...]  # remove dx
        end
        expr.args[end-2] = _expr.args[end-2]  # dx, x, p, t or x, p, t -> x
        expr.args[end-1] = _expr.args[end]  # dx, x, p, t or x, p, t -> t
        expr.args[end] = :integrator
        res = quote
            if @isdefined($:__LOGGER_DICT__)
                if $symbol == :__NESTED_LOG__
                    __LOGGER_DICT__ = $expr
                else
                    __LOGGER_DICT__[$symbol] = $expr
                end
            else
                $_expr
            end
        end
        esc(res)
    else
        error("Call a function for subsystem")
    end
end


# struct Integrator
#     p
# end

# function test()
#     @LOG function tmp(dx, x, p, t; q)
#         @log p
#         @log x_log = x
#         @log a, b = p, x
#         @log_only k = t^2  # activated only when logged
#         @log_only q  # activated only when logged
#         return nothing  # necessary; ignore the lasts line of the function body
#     end
#     @LOG function tmp2(dx, x, p, t; q)
#         @log p
#         @log x_log = x
#         @log a, b = p, x
#         @log_only k = t^2  # activated only when logged
#         @log_only q  # activated only when logged
#         return nothing  # necessary; ignore the lasts line of the function body
#     end
#     @LOG function foo(dx, x, p, t)
#         @nested_log :__NESTED_LOG__ tmp(dx, x, p, t; q=1)
#         @nested_log :subsystem tmp2(dx, x, p, t; q=1)
#         return x
#     end
#     x = [1, 2]
#     dx = zero.(x)
#     # p = [-1]
#     p = nothing
#     t = 0.0
#     integrator = Integrator(p)
#     return_raw = foo(dx, x, p, t)
#     @show return_raw
#     returndict = foo__LOG__(x, t, integrator)
#     @show returndict
#     nothing
# end
