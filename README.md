# FlightSims
`FlightSims` is a general-purpose numerical simulator by defining nested environments.
This package can be used for any kind of numerical simulation with dynamical systems
although it was supposed to be dedicated only for flight simulations.

## Features
### Compatibility
- It is compatible with [OrdinaryDiffEq.jl](https://github.com/SciML/OrdinaryDiffEq.jl).
Supporting full compatibility with [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl) is not on the road map for now.
If you want more functionality, please feel free to report an issue!
### Nested Environments and Zoo
- One can generate user-defined nested environments (or, dynamical systems) for complex flight simulation.
Also, some predefined environments are provided for reusability (i.e., environment zoo).
For more details, please see `src/Envs.jl` and `test/envs.jl`.
### Utilities
- Some utilities are also provided, for example, calculation of polynomial basis and 3D rotation.
For more details, please see `src/Utils.jl`.

## APIs
Main APIs are provided from `src/APIs.jl`.
### Make an environment
- `AbstractEnv`: an abstract type for user-defined and predefined environments.
All environment structures should be sub-type of `AbstractEnv`.
- `State(env::AbstractEnv)`: return a function that produces structured states.
- `dynamics(env::AbstractEnv)` and `dynamics!(env::AbstractEnv)`: return a function that maps out-of-place or in-place dynamics (resp.),
compatible with `DifferentialEquations`. User can extend these methods or simply define other methods.
### Simulation and data saving & loading
- [ ] To-do: Fill the following contents.
- `sim`: return `prob::ODEProblem` and `sol::ODESolution`.
- `process`: process `prob` and `sol` to get simulation data.
- `save`: save `env`, `prob`, `sol`, and optionally `process`,
in a `.jld2` file.
- `load`: load `env`, `prob`, `sol`, and optionally `process`,
from a `.jld2` file.

## Usage
- [ ] To-do: add an example
For now, see `test/sim_and_save.jl` and `test/load_and_plot.jl`.

## Issues
- [An issue](https://github.com/jonniedie/ComponentArrays.jl/issues/83)
has been reported; unknown crash between `OrdinaryDiffEq` and `ComponentArrays`.
