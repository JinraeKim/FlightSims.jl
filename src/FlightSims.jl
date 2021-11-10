module FlightSims

using Reexport
@reexport using FSimBase
using OrdinaryDiffEq: Tsit5  # ODEProblem default solver
using OrdinaryDiffEq: FunctionMap  # DiscreteProblem default solver
@reexport using FSimZoo


include("APIs/APIs.jl")


end
