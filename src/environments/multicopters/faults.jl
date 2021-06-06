abstract type AbstractFault end
abstract type AbstractActuatorFault <: AbstractFault end

# FaultSet
FaultSet(args...) = AbstractFault[args...]

# select_last
"""
Select the last fault (concerning the applied time) from given faults.
"""
function select_last_before_t(faults::Vector{AbstractFault}, t)
    faults_time_less_than_t = faults |> Map(fault -> fault.time) |> Filter(<=(t)) |> collect
    if length(faults_time_less_than_t) == 0
        # case 1: if there is no faults occured before t
        return (t, u) -> u
    else
        # case 2: if there are some faults occured before t
        faults_time_max = faults_time_less_than_t |> maximum
        fault_last_indices = findall(fault -> fault.time == maximum(faults_time_max), faults)
        fault_last_idx = length(fault_last_indices) == 1 ? fault_last_indices[1] : error("Faults more than one occured at the same time for the same index")
        return faults[fault_last_idx]
    end
end

(fault::AbstractFault)(t, u) = error("Not implemented fault")

"""
Loss of effectiveness (LoE).
"""
struct LoE <: AbstractActuatorFault
    time::Real
    index
    level::Real
    function LoE(time=0.0, index=1, level=1.0)
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
