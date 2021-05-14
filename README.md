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
Take a look at `src/environments`.
### Utilities
- Some utilities are also provided, for example, calculation of polynomial basis and 3D rotation.
Take a look at `src/utils`.

## APIs
Main APIs are provided in `src/APIs`.
### Make an environment
- `AbstractEnv`: an abstract type for user-defined and predefined environments.
All environment structures should be sub-type of `AbstractEnv`.
- `State(env::AbstractEnv)`: return a function that produces structured states.
- `dynamics(env::AbstractEnv)` and `dynamics!(env::AbstractEnv)`: return a function that maps out-of-place or in-place dynamics (resp.),
compatible with `DifferentialEquations`. User can extend these methods or simply define other methods.
- `apply_inputs(func; kwargs...)`: It is borrowed from [an MRAC example](https://jonniedie.github.io/ComponentArrays.jl/stable/examples/adaptive_control/). By using this, user can easily apply various kind of inputs into the dynamical system (environment).

### Simulation and data saving & loading
- [ ] To-do: Fill the following contents.
- `sim`: return `prob::ODEProblem` and `sol::ODESolution`.
- `process`: process `prob` and `sol` to get simulation data.
- `save`: save `env`, `prob`, `sol`, and optionally `process`,
in a `.jld2` file.
- `load`: load `env`, `prob`, `sol`, and optionally `process`,
from a `.jld2` file.

## Usage
- For an example of infinite-horizon continuous-time linear quadratic regulator (LQR), take a look at `test/lqr.jl`.
- [ ] To-do: add an example

## Issues
- [An issue](https://github.com/jonniedie/ComponentArrays.jl/issues/83)
has been reported; unknown crash between `OrdinaryDiffEq` and `ComponentArrays`.
Will be resolved by [this](https://github.com/jonniedie/ComponentArrays.jl/commit/0a6f1cfe41131a6b0ff2d26ad6bad5bc33f19671).
