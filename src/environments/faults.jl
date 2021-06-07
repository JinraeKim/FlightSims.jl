abstract type AbstractFault end
abstract type AbstractActuatorFault <: AbstractFault end

# FaultSet
FaultSet(args...) = AbstractFault[args...]

function select_last_before_t(faults, t)
    faults_before_t = faults |> Filter(fault -> fault.time <= t) |> collect
    if length(faults_before_t) == 0
        return (t, u) -> u
    else
        fault_times_before_t = faults_before_t |> Map(fault -> fault.time) |> collect
        faults_last_time = faults_before_t |> Filter(fault -> fault.time == maximum(fault_times_before_t)) |> collect
        fault_last_time = length(faults_last_time) == 1 ? faults_last_time[1] : error("More than one faults are applied at the same time")
        return fault_last_time
    end
end

## ActuatorFault
# LoE
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
    effectiveness[index] = level
    effectiveness .* u
end
