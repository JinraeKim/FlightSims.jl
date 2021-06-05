# multicopter
abstract type MulticopterEnv <: AbstractEnv end
# quadcopter
include("quadcopters.jl")
# mixer, faults
include("mixers.jl")
include("faults.jl")
