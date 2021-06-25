# FlightSims
[FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl) is a general-purpose numerical simulator supporting nested environments and convenient macro-based data logging.
## Plans and Changes
### v0.8
- [ ] find a good way of saving and loading simulation data
### v0.7
- [x] `df::DataFrame`, one of the outputs of `sim`, contains (nested) `NamedTuple`.
- [x] Separate logging tools as another package [SimulationLogger.jl](https://github.com/JinraeKim/SimulationLogger.jl).
    - [x] Previous logging tools, e.g., `Process` and `DatumFormat` have been deprecated.
### v0.6
- [x] Convenient logger will be added in `v0.6`; see [the related project](https://github.com/JinraeKim/FlightSims.jl/projects/4) and [#77](https://github.com/JinraeKim/FlightSims.jl/pull/77).
- [x] Default output of `sim` has been changed from `(prob::DEProblem, sol::DESolution)` to `(prob::DEProblem, df::DataFrame)`.
## Notes
### Why is it FlightSims.jl?
This package is for any kind of numerical simulation with dynamical systems
although it was supposed to be dedicated for flight simulations.
### Packages related to FlightSims.jl
- [SimulationLogger.jl](https://github.com/JinraeKim/SimulationLogger.jl): A convenient logging tools compatible with [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl).
- [FaultTolerantControl.jl](https://github.com/JinraeKim/FaultTolerantControl.jl):
fault tolerant control (FTC) with various models and algorithms of faults, fault detection and isolation (FDI), and reconfiguration (R) control.

## Features
### Compatibility
- It is highly based on [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl) but mainly focusing on ODE (ordinary differential equations).
- The construction of nested environments are based on [ComponentArrays.jl](https://github.com/jonniedie/ComponentArrays.jl).
- The structure of the resulting data from simulation result is based on [DataFrames.jl](https://github.com/JuliaData/DataFrames.jl).
- Logging tool is based on [SimulationLogger.jl](https://github.com/JinraeKim/SimulationLogger.jl).

If you want more functionality, please feel free to report an issue!

### Nested Environments and Zoo
- Environments usually stand for **dynamical systems** but also include **other utilities**, for example, controllers.
- One can generate user-defined nested environments using provided APIs.
Also, some predefined environments are provided for reusability (i.e., environment zoo).
Take a look at `src/environments`.
- Examples include
    - **basics**
        - (Linear system) `LinearSystemEnv`
        - (Reference model) `ReferenceModelEnv`
        - (Nonlinear system) `TwoDimensionalNonlinearPolynomialEnv`
            - [T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.](https://ieeexplore.ieee.org/document/7798777)
    - **multicopters**
        - (Quadcopter) `IslamQuadcopterEnv`, `GoodarziQuadcopterEnv`
        - (Hexacopter) `LeeHexacopterEnv`
    - **controllers**
        - (Linear quadratic regulator) `LQR`
    - **integrated_environments**
        - See `src/environments/integrated_environments`.

### Utilities
- Some utilities are also provided for dynamical system simulation.
- Examples include
    - **Function approximator**
        - (Approximator) `LinearApproximator`, `PolynomialBasis`
    - **Data manipulation for machine learning**
        - (Split data) `partitionTrainTest`
    - **Reference trajectory generator**
        - (Command generator) `HelixCommandGenerator`, `PowerLoop`
    - **Ridig body rotation**
        - (Rotations) `euler`

## APIs
Main APIs are provided in `src/APIs`.
Note that among APIs, most **[closures](https://docs.julialang.org/en/v1/devdocs/functions/#Closures) (a function whose output is a function)** will have the uppercase first letter ([#55](https://github.com/JinraeKim/FlightSims.jl/issues/55)).

### Make an environment
- `AbstractEnv`: an abstract type for user-defined and predefined environments.
In general, environments is a sub-type of `AbstractEnv`.
- `State(env::AbstractEnv)`: return a function that produces structured states.
- `Dynamics!(env::AbstractEnv)`, `Dynamics(env::AbstractEnv)`: return a function that maps in-place (**recommended**) and out-of-place dynamics (resp.),
compatible with [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl). User can extend these methods or simply define other methods.

Note that these interfaces are also provided for some **integrated environments**, e.g., `State(system, controller)`.

### Simulation, logging, and data saving & loading
**Core APIs**
- `sim`
    - return `prob::DEProblem` and `df::DataFrame`.
    - For now, only [**in-place** method (iip)](https://diffeq.sciml.ai/stable/basics/problem/#In-place-vs-Out-of-Place-Function-Definition-Forms) is supported.
- `apply_inputs(func; kwargs...)`
    - By using this, user can easily apply external inputs into environments. It is borrowed from [an MRAC example of ComponentArrays.jl](https://jonniedie.github.io/ComponentArrays.jl/stable/examples/adaptive_control/) and extended to be compatible with [SimulationLogger.jl](https://github.com/JinraeKim/SimulationLogger.jl).
- Macros for logging data: `@Loggable`, `@log`, `@onlylog`, `@nested_log`
    - For more details, see [SimulationLogger.jl](https://github.com/JinraeKim/SimulationLogger.jl).

*Deprecated APIs*
- `DatumFormat(env::AbstractEnv)`: return a function `(x, t, integrator::DiffEqBase.DEIntegrator) -> nt::NamedTuple` for saving data.
    - It is recommended users to use `DatumFormat(env::AbstractEnv)` for saving **basic information** of `env`.
    - Default setting: time and state histories will be saved as `df.time` and `df.state`.
- `save_inputs(func; kwargs...)`: this mimics `apply_inputs(func; kwargs...)`.
    - It is recommended users to use `save_inputs(func; kwargs...)` for saving **additional information**.
- `Process(env::AbstractEnv)`: return a function that processes `prob` and `sol` to get simulation data.
    - It is recommended users to use `Process(env::AbstractEnv)` when the simulation is **deterministic** (including parameter updates).
- `save`
    - Save `env`, `prob`, `sol`, and optionally `process`,
    - Not actively maintained. Please report issues about new features of saving data.
in a `.jld2` file.
- `load`
    - Load `env`, `prob`, `sol`, and optionally `process`,
from a `.jld2` file.
    - Not actively maintained. Please report issues about new features of loading data.

## Examples
### Optimal control and reinforcement learning
- For an example of **infinite-horizon continuous-time linear quadratic regulator (LQR)**,
see the following example code (`test/lqr.jl`).

```julia
using FlightSims
const FS = FlightSims
using DifferentialEquations
using LinearAlgebra
using Plots
using Test
using Transducers


function test()
    # linear system
    A = [0 1;
         0 0]
    B = [0;
         1]
    n, m = 2, 1
    env = LinearSystemEnv(A, B)  # exported from FlightSims
    x0 = State(env)([1.0, 2.0])
    p0 = zero.(x0)  # auxiliary parameter
    # optimal control
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    lqr = LQR(A, B, Q, R)  # exported from FlightSims
    u_lqr = FS.OptimalController(lqr)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞

    # simulation
    tf = 10.0
    Δt = 0.01
    affect!(integrator) = integrator.p = copy(integrator.u)  # auxiliary callback funciton
    cb = PeriodicCallback(affect!, Δt; initial_affect=true)  # auxiliary callback
    @Loggable function dynamics!(dx, x, p, t; u)
        @onlylog p  # activate this line only when logging data
        @log x, u
        @nested_log Dynamics!(env)(dx, x, p, t; u=u)  # exported `state` and `input` from `Dynamics!(env)`
    end
    prob, df = sim(
                   x0,  # initial condition
                   apply_inputs(dynamics!; u=u_lqr),  # dynamics with input of LQR
                   p0;
                   tf=tf,  # final time
                   callback=cb,
                   savestep=Δt,
                  )
    ts = df.time
    xs = df.sol |> Map(datum -> datum.x) |> collect
    us = df.sol |> Map(datum -> datum.u) |> collect
    ps = df.sol |> Map(datum -> datum.p) |> collect
    states = df.sol |> Map(datum -> datum.state) |> collect
    inputs = df.sol |> Map(datum -> datum.input) |> collect
    @test xs == states
    @test us == inputs
    p_x = plot(ts, hcat(states...)';
               title="state variable", label=["x1" "x2"], color=[:black :black], lw=1.5,
              )  # Plots
    plot!(p_x, ts, hcat(ps...)';
          ls=:dash, label="param", color=[:red :orange], lw=1.5
         )
    savefig("figures/x_lqr.png")
    plot(ts, hcat(inputs...)'; title="control input", label="u")  # Plots
    savefig("figures/u_lqr.png")
    df
end
```

```julia
julia> test()
1001×2 DataFrame
  Row │ time     sol
      │ Float64  NamedTup…
──────┼────────────────────────────────────────────
    1 │    0.0   (p = [1.01978, 1.95564], state =…
    2 │    0.01  (p = [1.01978, 1.95564], state =…
    3 │    0.02  (p = [1.03911, 1.91186], state =…
    4 │    0.03  (p = [1.05802, 1.86863], state =…
    5 │    0.04  (p = [1.07649, 1.82596], state =…
  ⋮   │    ⋮                     ⋮
  998 │    9.97  (p = [-0.00093419, 0.00103198], …
  999 │    9.98  (p = [-0.000923913, 0.00102347],…
 1000 │    9.99  (p = [-0.00091372, 0.001015], st…
 1001 │   10.0   (p = [-0.00091372, 0.001015], st…
                                   992 rows omitted
```

![ex_screenshot](./figures/x_lqr.png)
![ex_screenshot](./figures/u_lqr.png)

- For an example of **continuous-time value-iteration adaptive dynamic programming (CT-VI-ADP)**, take a look at `test/continuous_time_vi_adp.jl`.
    - [T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.](https://ieeexplore.ieee.org/document/7798777)
- For an example of **continuous-time integral reinforcement learning for linear system (CT-IRL)**, take a look at `test/continuous_time_linear_irl.jl`.
    - [F. L. Lewis, D. Vrabie, and K. G. Vamvoudakis, “Reinforcement Learning and Feedback Control: Using Natural Decision Methods to Design Optimal Adaptive Controllers,” IEEE Control Syst., vol. 32, no. 6, pp. 76–105, Dec. 2012, doi: 10.1109/MCS.2012.2214134.](https://d1wqtxts1xzle7.cloudfront.net/55631024/06315769.pdf?1516876343=&response-content-disposition=inline%3B+filename%3DUsing_natUral_decision_methods_to_design.pdf&Expires=1623395195&Signature=LP3BHxKg2mtIkqNFrR2C3NOOfxIxK6efgoHlXKFMH~IPjBL-Mi9CydRIhrqXQKOugpEaNAQR76H00mcz11ZoUbtTUUowVVWhYGk3iMK8aR~lUxO9b0A47iiJohLr6YpWhGm5AAgEDcKXa8DKFTAheBjGqTgFjL1Qm23MXlSXjWwR7DRhk5QtfiKjOQephv6c50CLinZxbz-VygOFTxuelbLcphrxuszszCVLZtS0K0sH~3f9RZkIJcNKqe8t18ghkHxfSZTapae0AZSslGaGLjBlbqF9RSCc04eQZorZmHxvrYd4CZ0Zac7Hn3M3--Qe81tL-32ULl~XLYk1Q5Ev4A__&Key-Pair-Id=APKAJLOHF5GGSLRBV4ZA)

### Nonlinear control
- For an example of **backstepping position tracking controller for quadcopters**,
visit [FaultTolerantControl.jl](https://github.com/JinraeKim/FaultTolerantControl.jl).

### Scientific machine learning
- [ ] Add examples for newbies!
- For an example usage of [Flux.jl](https://github.com/FluxML/Flux.jl), see `main/flux_example.jl`.
- For an example code of an imitation learning algorithm, behavioural cloning, see `main/behavioural_cloning.jl`.
