module FlightSims

# using Debugger  # tmp
using Reexport
using Requires  # to enhance startup time
using DifferentialEquations
using SimulationLogger
using ComponentArrays, UnPack
using JLD2, FileIO, DataFrames
using NamedTupleTools
using LinearAlgebra, Transducers, Random, ForwardDiff, ReferenceFrameRotations
using MatrixEquations
using Combinatorics: multiexponents
using NLopt
using NumericalIntegration: integrate
using DynamicPolynomials: @polyvar, PolyVar, AbstractPolynomialLike
using Flux
using Flux.Data: DataLoader
using Plots  # tmp

### APIs
export AbstractEnv, State, Params, Dynamics, Dynamics!
export sim, apply_inputs, Command
@reexport using SimulationLogger
### envs
## basics
export TwoDimensionalNonlinearPolynomialEnv, LinearSystemEnv, ReferenceModelEnv, MultipleEnvs
## controllers
export LQR, PID
## multicopters
export MulticopterEnv
export QuadcopterEnv, IslamQuadcopterEnv, GoodarziQuadcopterEnv
export HexacopterEnv, LeeHexacopterEnv
# control allocator
export AbstractAllocator, StaticAllocator
export PseudoInverseAllocator
### algorithms
# export command
export CTValueIterationADP, BehaviouralCloning, CTLinearIRL
# utils
export AbstractApproximator, LinearApproximator
export PolynomialBasis, euler
export PowerLoop, HelixCommandGenerator
export ned2enu


include("APIs/APIs.jl")
include("utils/utils.jl")
include("environments/environments.jl")
include("algorithms/algorithms.jl")

include("render.jl")  # tmp


# function __init__()
#     @require Plots = "91a5bcdd-55d7-5caf-9e0b-520d859cae80" include("render.jl")
# end


end
