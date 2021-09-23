module FlightSims

using Reexport
@reexport using FSimBase
using OrdinaryDiffEq: Tsit5  # default solver
# using Plots

using ComponentArrays, UnPack
using LinearAlgebra, MatrixEquations, ReferenceFrameRotations, ForwardDiff  # dependencies of hexacopter position control
using Transducers
# using Random
# using Combinatorics: multiexponents
# using NLopt
# using NumericalIntegration: integrate
# using DynamicPolynomials: @polyvar, PolyVar, AbstractPolynomialLike

### APIs
export sim
### envs
## basics
export TwoDimensionalNonlinearPolynomialEnv, LinearSystemEnv, ReferenceModelEnv, MultipleEnvs
## controllers
export LQR, PID, BacksteppingPositionControllerEnv
export PPNG
## multicopters
export MulticopterEnv
export QuadcopterEnv, IslamQuadcopterEnv, GoodarziQuadcopterEnv
export HexacopterEnv, LeeHexacopterEnv
## missiles
export PointMass3DMissile, PursuerEvador3DMissile
# control allocator
export AbstractAllocator, StaticAllocator
export PseudoInverseAllocator
### algorithms
# # export command
# export CTValueIterationADP, BehaviouralCloning, CTLinearIRL
# # utils
# export AbstractApproximator, LinearApproximator
# export PolynomialBasis
export PowerLoop, HelixCommandGenerator
export ned2enu, enu2ned


include("APIs/APIs.jl")
include("utils/utils.jl")
include("environments/environments.jl")
# include("algorithms/algorithms.jl")
# include("render.jl")


end
