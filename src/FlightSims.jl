module FlightSims

using Reexport
@reexport using FSimBase
@reexport using FSimPlots
using OrdinaryDiffEq: Tsit5  # default solver

using ComponentArrays, UnPack
using LinearAlgebra, MatrixEquations, ReferenceFrameRotations, ForwardDiff  # dependencies of hexacopter position control
using Transducers

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
export PowerLoop, HelixCommandGenerator
export ned2enu, enu2ned


include("APIs/APIs.jl")
include("utils/utils.jl")
include("environments/environments.jl")


end
