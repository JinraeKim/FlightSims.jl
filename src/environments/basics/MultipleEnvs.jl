struct MultipleEnvs{T <: AbstractEnv, S <: Union{Symbol, String}} <: AbstractEnv
    envs_dict::Dict{S, T}
end

function State(envs::MultipleEnvs)
    @unpack envs_dict = envs
    function state(; state_pairs...)
        nt = (; state_pairs...)
        ComponentArray(nt)
    end
end

function Dynamics!(envs::MultipleEnvs)
    @unpack envs_dict = envs
    @Loggable function dynamics!(dx, x, p, t; kwargs_pairs...)
        for (key, kwargs) in kwargs_pairs
            @nested_log key Dynamics!(envs_dict[key])(getproperty(dx, key),
                                                      getproperty(x, key),
                                                      hasproperty(p, key) ? getproperty(p, key) : p,
                                                      t;
                                                      kwargs...)
        end
    end
end
