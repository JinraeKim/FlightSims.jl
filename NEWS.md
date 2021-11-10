# Changes in v1.0.0
- Add `Simulator` interface for both interactive and non-interactive simulation.
- Add [FSimROS.jl](https://github.com/JinraeKim/FSimROS.jl).

# Changes in v0.10.0
- Predefined environments and controllers are detached as [FSimZoo.jl](https://github.com/JinraeKim/FSimZoo.jl). Some functionalities has been deprecated, e.g., `CommandGenerator`.
- Predefined stuffs' names are changed.

# Changes in v0.9.0
- Since v0.9.0, the core functionality of FlightSims.jl has been detached as [FSimBase.jl](https://github.com/JinraeKim/FSimBase.jl), which also works alone!
- Since v0.9.0, the plotting tools for predefined environments has been detached as [FSimPlots.jl](https://github.com/JinraeKim/FSimPlots.jl).
- Some sub-directories, e.g., `src/algorithms`, `src/utils/approximators` have been deprecated. They will be detached as new packages if necessary.

