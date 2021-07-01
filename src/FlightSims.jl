module FlightSims

# using Debugger  # tmp
using Reexport
using DifferentialEquations
using SimulationLogger
using ComponentArrays, UnPack
using JLD2, FileIO, DataFrames
using NamedTupleTools
using LinearAlgebra, Transducers, Random, ForwardDiff
using MatrixEquations
using Combinatorics: multiexponents
using NLopt
using NumericalIntegration: integrate
using DynamicPolynomials: @polyvar, PolyVar, AbstractPolynomialLike
using Flux
using Flux.Data: DataLoader

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


include("APIs/APIs.jl")
include("utils/utils.jl")
include("environments/environments.jl")
include("algorithms/algorithms.jl")


end
