using MacroTools


struct LOG_INDICATOR
end

macro Loggable(defun)
    _def = splitdef(defun)  # original
    def = deepcopy(_def)
    _body = _def[:body]
    args = _def[:args]
    args = push!(args, :(__LOG_INDICATOR__::LOG_INDICATOR))
    def[:body] = quote
        __LOGGER_DICT__ = @isdefined($(:__LOGGER_DICT__)) ? __LOGGER_DICT__ : Dict()
        $(args[end-2]) = copy($(args[end-2]))  # dx, x, p, t or x, p, t -> x (copy for view issue: https://diffeq.sciml.ai/stable/features/callback_library/#Constructor-5)
        $(_body.args[1:end]...)  # remove the last line, return, to return __LOGGER_DICT__ 
        return __LOGGER_DICT__  # Dictionary (see `sim`; just a convention)
    end
    res = quote
        # $(MacroTools.combinedef(_def))
        $(MacroTools.combinedef(def))
    end
    esc(res)
end

macro log(expr)
    esc(:(@isdefined($:__LOGGER_DICT__) ? @log($:__LOGGER_DICT__, $expr) : $expr))
end
macro onlylog(expr)
    esc(:(@isdefined($:__LOGGER_DICT__) ? @log($:__LOGGER_DICT__, $expr) : nothing))
end

macro log(__LOGGER_DICT__, expr)
    return if expr isa Symbol
        quote
            # local val = $(:(copy($(esc(expr)))))
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
        # _expr = copy(expr)
        # expr.args[1] = Symbol(String(_expr.args[1]) * String(:__LOG__))  # function name (e.g., my_func -> my_func__LOG__)
        # if length(expr.args) >= 5 && typeof(expr.args[end-3]) == Symbol
        #     expr.args = [expr.args[1:end-4]..., expr.args[end-2:end]...]  # remove dx
        # end
        # expr.args[end-2] = _expr.args[end-2]  # dx, x, p, t or x, p, t -> x
        # expr.args[end-1] = _expr.args[end]  # dx, x, p, t or x, p, t -> t
        # expr.args[end] = :integrator
        res = quote
            if @isdefined($:__LOGGER_DICT__)
                if $symbol == :__NESTED_LOG__
                    __LOGGER_DICT__ = $expr
                else
                    __LOGGER_DICT__[$symbol] = $expr
                end
            else
                $expr
            end
        end
        esc(res)
    else
        error("Call the ODE function of a sub-environment, e.g., `@nested_log :env_name dynamics!(dx.sub, x.sub, p.sub, t)`")
    end
end


function test()
    @Loggable function dynamics!(dx, x, p, t)
        dx .= -x
    end
end
