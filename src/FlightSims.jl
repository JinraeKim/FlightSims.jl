module FlightSims

# using Debugger  # tmp
# using OrdinaryDiffEq
using DifferentialEquations
using ComponentArrays, UnPack
using JLD2, FileIO, DataFrames
using LinearAlgebra, Transducers, Rotations, Random, ForwardDiff
using Combinatorics: multiexponents
using NLopt
using NumericalIntegration: integrate
using DynamicPolynomials: @polyvar, PolyVar, AbstractPolynomialLike
using Flux
using Flux.Data: DataLoader
using MatrixEquations

# APIs
export AbstractEnv, State, dynamics, dynamics!, apply_inputs
export sim, process, load
# envs
export TwoDimensionalNonlinearPolynomialEnv, LinearSystemEnv, ReferenceModelEnv
export IslamQuadcopterEnv, GoodarziQuadcopterEnv
export BacksteppingPositionControllerEnv
# algorithms
export command
export CTValueIterationADP, BehaviouralCloning
# utils
export AbstractApproximator, LinearApproximator
export PolynomialBasis, euler
export PowerLoop, command_generator


include("environments/environments.jl")
include("APIs/APIs.jl")
include("utils/utils.jl")
include("algorithms/algorithms.jl")


end
