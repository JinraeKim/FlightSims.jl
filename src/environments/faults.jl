abstract type AbstractFault end
abstract type AbstractActuatorFault <: AbstractFault end

# API
(fault::AbstractFault)(t, u) = error("Not implemented fault")
# FaultSet
FaultSet(args...) = AbstractFault[args...]
# select_last
"""
Select the last fault (concerning the applied time) among given faults.
"""
function select_last_before_t(faults::Vector{T} where T <: AbstractFault, t)
    fault_times_less_than_t = faults |> Map(fault -> fault.time) |> Filter(<=(t)) |> collect
    if length(fault_times_less_than_t) == 0
        # case 1: if there is no faults occured before t
        return (t, u) -> u
    else
        # case 2: if there are some faults occured before t
        fault_last_indices = findall(fault -> fault.time == maximum(fault_times_less_than_t), faults)
        fault_last_idx = length(fault_last_indices) == 1 ? fault_last_indices[1] : error("Among given faults, more than one faults occured at the same time")
        return faults[fault_last_idx]
    end
end


"""
Loss of effectiveness (LoE).
"""
struct LoE <: AbstractActuatorFault
    time::Real
    index
    level::Real
    function LoE(time, index, level)
        @assert level >= 0.0 && level <= 1.0
        new(time, index, level)
    end
end

function (fault::LoE)(t, u)
    @unpack time, index, level = fault
    effectiveness = ones(size(u))
    if t >= time
        effectiveness[index] = level
    end
    return effectiveness .* u
end
