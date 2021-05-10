# FlightSims
`FlightSims` is a general-purpose numerical simulator by defining nested environments.
It is originally dedicated for flight simulations, but it can be used for any kind of numerical simulation with dynamical systems.

## Features
### Compatibility
- It is compatible with other packages including [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl).
### Nested Environments
- One can generate user-defined nested environments for complex flight simulation.
### Environment zoo
- One can reuse predefined environments.
### Utilities
- Some utilities are also provided including polynomial basis.

## APIs
The following description may describe the default behaviour. 
### Make an environment
- `AbstractEnv`: an abstract type for user-defined and predefined environments
- `State(env::AbstractEnv)`: return a function that maps the input to a structured state.
- `dynamics(env::AbstractEnv)` and `dynamics!(env::AbstractEnv)`: return a function that maps out-of-place or in-place dynamics (resp.), compatible with `DifferentialEquations`.
### Simulation and data saving & loading
- [ ] To-do: Fill the following contents.
- `sim`
- `process`
- `save`
- `load`

## Usage
- [ ] To-do: add an example
For now, see `test/sim_and_save.jl` and `test/load_and_plot.jl`.

## Issues
- [An issue](https://github.com/jonniedie/ComponentArrays.jl/issues/83)
has been reported; unknown crash between `OrdinaryDiffEq` and `ComponentArrays`.
