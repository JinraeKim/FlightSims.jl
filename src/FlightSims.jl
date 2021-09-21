module FlightSims

# using Debugger  # tmp
using Reexport
using Plots
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

### APIs
export AbstractEnv, State, Params, Dynamics, Dynamics!
export sim, apply_inputs, Command
@reexport using SimulationLogger
### envs
## basics
export TwoDimensionalNonlinearPolynomialEnv, LinearSystemEnv, ReferenceModelEnv, MultipleEnvs
## controllers
export LQR, PID, BacksteppingPositionControllerEnv
export PPNG, BPNG
## multicopters
export MulticopterEnv
export QuadcopterEnv, IslamQuadcopterEnv, GoodarziQuadcopterEnv
export HexacopterEnv, LeeHexacopterEnv
## missiles
export PointMass2DMissile, PursuerEvador2DMissile
export PointMass3DMissile, PursuerEvador3DMissile
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
export ned2enu, enu2ned
export fig_print


include("APIs/APIs.jl")
include("utils/utils.jl")
include("environments/environments.jl")
include("algorithms/algorithms.jl")
include("render.jl")


end
