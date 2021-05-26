module FlightSims

# using Debugger  # tmp
using OrdinaryDiffEq
using ComponentArrays, UnPack
using JLD2, FileIO, DataFrames
using LinearAlgebra, Transducers, Rotations
using Combinatorics: multiexponents
using NLopt
using NumericalIntegration: integrate
using DynamicPolynomials: @polyvar, PolyVar, AbstractPolynomialLike

export AbstractEnv, State, dynamics, dynamics!, apply_inputs
export sim, process, load
export TwoDimensionalNonlinearPolynomialEnv, LinearSystemEnv
export GoodarziQuadcopterEnv
export AbstractApproximator, LinearApproximator
export PolynomialBasis, euler
export CTValueIterationADP


include("environments/environments.jl")
include("APIs/APIs.jl")
include("utils/utils.jl")
include("algorithms/algorithms.jl")


end
