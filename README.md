# FlightSims
[FlightSims.jl](https://github.com/JinraeKim/FlightSims.jl) is a general-purpose numerical simulator supporting nested environments and convenient macro-based data logging.
## Plans and Changes
### v0.6
- [x] Convenient logger will be added in `v0.6`; see [the related project](https://github.com/JinraeKim/FlightSims.jl/projects/4) and [#77](https://github.com/JinraeKim/FlightSims.jl/pull/77).
- [x] Default output of `sim` has been changed from `(prob::DEProblem, sol::DESolution)` to `(prob::DEProblem, df::DataFrame)`.
### v0.5
- [x] Some controllers and utilities will be separated in `v0.5`; see [FaultTolerantControl.jl](https://github.com/JinraeKim/FaultTolerantControl.jl).
## Notes
### Why is it FlightSims.jl?
This package is for any kind of numerical simulation with dynamical systems
although it was supposed to be dedicated for flight simulations.
### Packages based on FlightSims.jl
- [FaultTolerantControl.jl](https://github.com/JinraeKim/FaultTolerantControl.jl):
fault tolerant control (FTC) with various models and algorithms of faults, fault detection and isolation (FDI), and reconfiguration (R) control.

## Features
### Compatibility
- It is highly based on [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl) but mainly focusing on ODE (ordinary differential equations).
- The construction of nested environments are based on [ComponentArrays.jl](https://github.com/jonniedie/ComponentArrays.jl).
- The structure of the resulting data from simulation result is based on [DataFrames.jl](https://github.com/JuliaData/DataFrames.jl).

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
    <!-- - With a keyword argument `datum_format`, it results `prob`, `sol`, and `df::DataFrame`. -->
- `apply_inputs(func; kwargs...)`
    - By using this, user can easily apply external inputs into environments. It is borrowed from [an MRAC example of ComponentArrays.jl](https://jonniedie.github.io/ComponentArrays.jl/stable/examples/adaptive_control/).
- `@Loggable`
    - (usage: `@Loggable function my_func(dx, x, p, t) #blahblah end`) make your ODE function loggable. Use this macro when **defining** your ODEFunction. This actually makes a hidden dictionary and return it.
    - **DO NOT** use `return` syntax in the function annotated by `@Loggable`. Instead, just mutate `dx` or simply leave any result without `return` as
        ```julia
        @Loggable function my_func(dx, x, p, t)
            dx .= x
            nothing  # `return nothing` would yield undesirable behaviour. Actually, you can simply remove this line. 
        end
        ```

    - `__LOGGER_DICT__`:  This is an alias of the hidden dictionary generated by `@Loggable`. **NEVER USE THE NAME `__LOGGER_DICT__` in usual cases**.
- `@log`
    - (usage: `@log var_name = val`) variables annotated by this macro will be logged (Actually this stands for `__LOGGER_DICT__[var_name] = val`).
- `@onlylog`
    - The same as `@log` but it will be activated only when logging variables. It is not activated when solving DEProblem.
    - This basic form of this macro is inspired by [SimulationLogs.jl](https://github.com/jonniedie/SimulationLogs.jl). But there are some differences. For example, `@log` in this package is based on [SavingCallback](https://diffeq.sciml.ai/stable/features/callback_library/#saving_callback), while `@log` in [SimulationLogs.jl](https://github.com/jonniedie/SimulationLogs.jl) will save data in the sense of postprocessing.
- `@nested_log`
    - (usage 1) `@nested_log env_name ODEFunction_call`): this macro will save all variables logged in `ODEFunction_call` as `__LOGGER_DICT__[env_name]`. `ODEFunction_call` should be annotated by `@Loggable`.
    - (usage 2) `@nested_log env_name var_name=val`): this macro will save value `val` as `var_name` in a nested sense (at `env_name`).

*Will be deprecated*
- `DatumFormat(env::AbstractEnv)`: return a function `(x, t, integrator::DiffEqBase.DEIntegrator) -> nt::NamedTuple` for saving data.
    - It is recommended users to use `DatumFormat(env::AbstractEnv)` for saving **basic information** of `env`.
    - Default setting: time and state histories will be saved as `df.time` and `df.state`.
- `save_inputs(func; kwargs...)`: this mimics `apply_inputs(func; kwargs...)`.
    - It is recommended users to use `save_inputs(func; kwargs...)` for saving **additional information**.
- `Process(env::AbstractEnv)`: return a function that processes `prob` and `sol` to get simulation data.
    - It is recommended users to use `Process(env::AbstractEnv)` when the simulation is **deterministic** (including parameter updates).

*Not actively maintained*
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
    affect!(integrator) = integrator.p = copy(integrator.u)  # auxiliary callback
    cb = PeriodicCallback(affect!, Δt; initial_affect=true)
    @Loggable function dynamics!(dx, x, p, t; u)
        @onlylog p  # activate this line only when logging data
        @log state = x
        @log input = u
        # nested logging
        @nested_log :linear state_square = x .^ 2  # to put additional data into the same symbol (:linear)
        @nested_log :linear Dynamics!(env)(dx, x, p, t; u=u)
    end
    prob, df = sim(
                   x0,  # initial condition
                   # apply_inputs(Dynamics!(env); u=u_lqr),  # dynamics with input of LQR
                   apply_inputs(dynamics!; u=u_lqr),  # dynamics with input of LQR
                   p0;
                   tf=tf,  # final time
                   callback=cb,
                   savestep=Δt,
                  )
    p_x = plot(df.time, hcat(df.state...)';
               title="state variable", label=["x1" "x2"], color=[:black :black], lw=1.5,
              )  # Plots
    plot!(p_x, df.time, hcat(df.p...)';
          ls=:dash, label="param", color=[:red :orange], lw=1.5
         )
    savefig("figures/x_lqr.png")
    plot(df.time, hcat(df.input...)'; title="control input", label="u")  # Plots
    savefig("figures/u_lqr.png")
    df
end
```

```julia
julia> test()
1001×5 DataFrame
  Row │ time     p                           state                       linear                             input
      │ Float64  Array…                      Array…                      Dict…                              Array…
──────┼────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
    1 │    0.0   [1.01978, 1.95564]          [1.0, 2.0]                  Dict{Any, Any}(:state_square=>[1…  [-4.4641]
    2 │    0.01  [1.01978, 1.95564]          [1.01978, 1.95564]          Dict{Any, Any}(:state_square=>[1…  [-4.40705]
    3 │    0.02  [1.03911, 1.91186]          [1.03911, 1.91186]          Dict{Any, Any}(:state_square=>[1…  [-4.35055]
    4 │    0.03  [1.05802, 1.86863]          [1.05802, 1.86863]          Dict{Any, Any}(:state_square=>[1…  [-4.29458]
    5 │    0.04  [1.07649, 1.82596]          [1.07649, 1.82596]          Dict{Any, Any}(:state_square=>[1…  [-4.23915]
    6 │    0.05  [1.09454, 1.78385]          [1.09454, 1.78385]          Dict{Any, Any}(:state_square=>[1…  [-4.18425]
    7 │    0.06  [1.11217, 1.74228]          [1.11217, 1.74228]          Dict{Any, Any}(:state_square=>[1…  [-4.12988]
    8 │    0.07  [1.12939, 1.70125]          [1.12939, 1.70125]          Dict{Any, Any}(:state_square=>[1…  [-4.07603]
    9 │    0.08  [1.14619, 1.66075]          [1.14619, 1.66075]          Dict{Any, Any}(:state_square=>[1…  [-4.02271]
   10 │    0.09  [1.1626, 1.62079]           [1.1626, 1.62079]           Dict{Any, Any}(:state_square=>[1…  [-3.9699]
   11 │    0.1   [1.17861, 1.58135]          [1.17861, 1.58135]          Dict{Any, Any}(:state_square=>[1…  [-3.9176]
   12 │    0.11  [1.19423, 1.54244]          [1.19423, 1.54244]          Dict{Any, Any}(:state_square=>[1…  [-3.86581]
   13 │    0.12  [1.20946, 1.50404]          [1.20946, 1.50404]          Dict{Any, Any}(:state_square=>[1…  [-3.81453]
   14 │    0.13  [1.22431, 1.46615]          [1.22431, 1.46615]          Dict{Any, Any}(:state_square=>[1…  [-3.76375]
   15 │    0.14  [1.23879, 1.42876]          [1.23879, 1.42876]          Dict{Any, Any}(:state_square=>[1…  [-3.71347]
   16 │    0.15  [1.25289, 1.39187]          [1.25289, 1.39187]          Dict{Any, Any}(:state_square=>[1…  [-3.66369]
   17 │    0.16  [1.26663, 1.35548]          [1.26663, 1.35548]          Dict{Any, Any}(:state_square=>[1…  [-3.6144]
   18 │    0.17  [1.28, 1.31959]             [1.28, 1.31959]             Dict{Any, Any}(:state_square=>[1…  [-3.56559]
   19 │    0.18  [1.29302, 1.28417]          [1.29302, 1.28417]          Dict{Any, Any}(:state_square=>[1…  [-3.51727]
   20 │    0.19  [1.30569, 1.24924]          [1.30569, 1.24924]          Dict{Any, Any}(:state_square=>[1…  [-3.46943]
   21 │    0.2   [1.31801, 1.21478]          [1.31801, 1.21478]          Dict{Any, Any}(:state_square=>[1…  [-3.42207]
   22 │    0.21  [1.32998, 1.1808]           [1.32998, 1.1808]           Dict{Any, Any}(:state_square=>[1…  [-3.37518]
   23 │    0.22  [1.34162, 1.14728]          [1.34162, 1.14728]          Dict{Any, Any}(:state_square=>[1…  [-3.32876]
  ⋮   │    ⋮                 ⋮                           ⋮                               ⋮                        ⋮
  979 │    9.78  [-0.00114617, 0.00120202]   [-0.00114617, 0.00120202]   Dict{Any, Any}(:state_square=>[1…  [-0.00093579]
  980 │    9.79  [-0.0011342, 0.00119268]    [-0.0011342, 0.00119268]    Dict{Any, Any}(:state_square=>[1…  [-0.000931591]
  981 │    9.8   [-0.00112232, 0.00118339]   [-0.00112232, 0.00118339]   Dict{Any, Any}(:state_square=>[1…  [-0.000927372]
  982 │    9.81  [-0.00111053, 0.00117414]   [-0.00111053, 0.00117414]   Dict{Any, Any}(:state_square=>[1…  [-0.000923134]
  983 │    9.82  [-0.00109883, 0.00116493]   [-0.00109883, 0.00116493]   Dict{Any, Any}(:state_square=>[1…  [-0.000918877]
  984 │    9.83  [-0.00108723, 0.00115576]   [-0.00108723, 0.00115576]   Dict{Any, Any}(:state_square=>[1…  [-0.000914602]
  985 │    9.84  [-0.00107572, 0.00114663]   [-0.00107572, 0.00114663]   Dict{Any, Any}(:state_square=>[1…  [-0.00091031]
  986 │    9.85  [-0.0010643, 0.00113755]    [-0.0010643, 0.00113755]    Dict{Any, Any}(:state_square=>[1…  [-0.000906001]
  987 │    9.86  [-0.00105297, 0.00112851]   [-0.00105297, 0.00112851]   Dict{Any, Any}(:state_square=>[1…  [-0.000901676]
  988 │    9.87  [-0.00104173, 0.00111952]   [-0.00104173, 0.00111952]   Dict{Any, Any}(:state_square=>[1…  [-0.000897337]
  989 │    9.88  [-0.00103058, 0.00111057]   [-0.00103058, 0.00111057]   Dict{Any, Any}(:state_square=>[1…  [-0.000892982]
  990 │    9.89  [-0.00101952, 0.00110166]   [-0.00101952, 0.00110166]   Dict{Any, Any}(:state_square=>[1…  [-0.000888614]
  991 │    9.9   [-0.00100854, 0.0010928]    [-0.00100854, 0.0010928]    Dict{Any, Any}(:state_square=>[1…  [-0.000884233]
  992 │    9.91  [-0.000997661, 0.00108398]  [-0.000997661, 0.00108398]  Dict{Any, Any}(:state_square=>[9…  [-0.00087984]
  993 │    9.92  [-0.000986865, 0.0010752]   [-0.000986865, 0.0010752]   Dict{Any, Any}(:state_square=>[9…  [-0.000875434]
  994 │    9.93  [-0.000976157, 0.00106647]  [-0.000976157, 0.00106647]  Dict{Any, Any}(:state_square=>[9…  [-0.000871018]
  995 │    9.94  [-0.000965535, 0.00105778]  [-0.000965535, 0.00105778]  Dict{Any, Any}(:state_square=>[9…  [-0.000866591]
  996 │    9.95  [-0.000955001, 0.00104913]  [-0.000955001, 0.00104913]  Dict{Any, Any}(:state_square=>[9…  [-0.000862154]
  997 │    9.96  [-0.000944553, 0.00104054]  [-0.000944553, 0.00104054]  Dict{Any, Any}(:state_square=>[8…  [-0.000857708]
  998 │    9.97  [-0.00093419, 0.00103198]   [-0.00093419, 0.00103198]   Dict{Any, Any}(:state_square=>[8…  [-0.000853253]
  999 │    9.98  [-0.000923913, 0.00102347]  [-0.000923913, 0.00102347]  Dict{Any, Any}(:state_square=>[8…  [-0.00084879]
 1000 │    9.99  [-0.00091372, 0.001015]     [-0.00091372, 0.001015]     Dict{Any, Any}(:state_square=>[8…  [-0.00084432]
 1001 │   10.0   [-0.00091372, 0.001015]     [-0.000903612, 0.00100658]  Dict{Any, Any}(:state_square=>[8…  [-0.000839842]
                                                                                                           955 rows omitted
```

![ex_screenshot](./figures/x_lqr.png)
![ex_screenshot](./figures/u_lqr.png)

- For an example of **nested environments and nested logging**,
see the following example code (`test/nested_envs.jl`).

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
