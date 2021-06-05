abstract type AbstractFault end
abstract type AbstractActuatorFault <: AbstractFault end

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
