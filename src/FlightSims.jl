module FlightSims

using OrdinaryDiffEq
using ComponentArrays
using UnPack
using DataFrames
using JLD2, FileIO
using Transducers
using Combinatorics
using LinearAlgebra, Rotations

export AbstractEnv, State, dynamics, dynamics!, apply_inputs
export sim, process, load
export TwoDimensionalNonlinearPolynomialEnv,
       GoodarziQuadcopterEnv
export AbstractApproximator, LinearApproximator
export polynomial_basis, euler
export CTValueIterationADP


include("environments/environments.jl")
include("APIs/APIs.jl")
include("utils/utils.jl")
include("algorithms/algorithms.jl")


end
