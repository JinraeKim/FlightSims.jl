### Types
abstract type AbstractEnv end

### faults
include("faults.jl")
### FDI (Fault Detection and Isoltion)
include("FDI.jl")

### Envs
## Basic environments
include("basics/basics.jl")

## Multicopters
include("multicopters/multicopters.jl")

## Control Allocator
include("allocators/allocators.jl")

## Controllers
include("controllers/controllers.jl")

## Integrated Envs
include("integrated_environments/integrated_environments.jl")
