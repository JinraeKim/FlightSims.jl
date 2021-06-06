# multicopter
abstract type MulticopterEnv <: AbstractEnv end
# mixer, faults
include("mixers.jl")
include("faults.jl")
# quadcopter
include("quadcopters.jl")

