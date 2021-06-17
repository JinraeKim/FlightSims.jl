macro LOG(defun)
    _def = splitdef(defun)  # original
    def = copy(_def)
    def[:name] = Symbol(String(_def[:name]) * String(:__LOG__))
    body = _def[:body]
    args = _def[:args]
    dx_expr = args[1]
    x_expr = args[2]
    p_expr = args[3]
    t_expr = args[4]
    def[:args] = (x_expr, t_expr, :integrator)
    def[:body] = quote
        __LOGGER_DICT__ = Dict()
        $(args[end-2]) = copy($(def[:args][1]))  # dx, x, p, t or x, p, t -> x
        $(args[end-1]) = copy(getfield($(def[:args][3]), :p))  # dx, x, p, t or x, p, t -> p
        $(args[end]) = copy($(def[:args][2]))  # dx, x, p, t or x, p, t -> t
        $(body.args[1:end-1]...)  # remove return
        return __LOGGER_DICT__
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
            if !haskey(logger_dict, var_name)
                logger_dict[var_name] = typeof(val)[]
            end
            push!(getindex(logger_dict, var_name), val)
            nothing
        end
    elseif expr.head == :(=)
        if expr.args[1] isa Expr && expr.args[1].head == :tuple
            quote
                local vals = $(esc(expr.args[2]))
                local var_names = $(esc(expr.args[1].args))
                local logger_dict = $(esc(__LOGGER_DICT__))
                for (var_name, val) in zip(var_names, vals)
                    if !haskey(logger_dict, var_name)
                        logger_dict[var_name] = typeof(val)[]
                    end
                    push!(getindex(logger_dict, var_name), val)
                end
                $(esc(expr))
            end
        else
            quote
                local val = $(esc(expr.args[2]))
                local var_name = $((expr.args[1],))[1]
                local logger_dict = $(esc(__LOGGER_DICT__))
                if !haskey(logger_dict, var_name)
                    logger_dict[var_name] = typeof(val)[]
                end
                push!(getindex(logger_dict, var_name), val)
                $(esc(expr))
            end
        end
    else
        :(error("To log a variable, use either one of forms: `@log val` or `@log var_name = val`"))
    end
end
