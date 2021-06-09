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
- Environments usually stand for **dynamical systems** but also contain **other utilities**, for example, controllers.
- One can generate user-defined nested environments using provided APIs.
Also, some predefined environments are provided for reusability (i.e., environment zoo).
Take a look at `src/environments`.
- Examples include
    - **basics**
        - (Linear system) `LinearSystemEnv`
        - (Reference model) `ReferenceModelEnv`
    - **multicopters**
        - (Quadcopter) `IslamQuadcopterEnv`, `GoodarziQuadcopterEnv`
    - **controllers**
        - (Linear quadratic regulator) `LQR`
        - (Backstepping controller) `BacksteppingPositionControllerEnv`
    - **integrated_environments**
    - others
        - (Actuator fault) `LoE`

### Utilities
- Some utilities are also provided, for example, calculation of polynomial basis and 3D rotation.
Take a look at `src/utils`.
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
Note that among APIs, "a function whose output is a function" will have the uppercase first letter (#55).

### Make an environment
- `AbstractEnv`: an abstract type for user-defined and predefined environments.
All environment structures should be sub-type of `AbstractEnv`.
- `State(env::AbstractEnv)`: return a function that produces structured states.
- `Dynamics!(env::AbstractEnv)` and `Dynamics(env::AbstractEnv)`: return a function that maps in-place (**recommended**) and out-of-place dynamics (resp.),
compatible with `DifferentialEquations`. User can extend these methods or simply define other methods.
- `apply_inputs(func; kwargs...)`: It is borrowed from [an MRAC example](https://jonniedie.github.io/ComponentArrays.jl/stable/examples/adaptive_control/). By using this, user can easily apply various kind of inputs into the dynamical system (environment).

Note that these interfaces are also provided for some **integrated environments**, e.g., `State(system, controller)`.

### Simulation and data saving & loading
- `sim`: return `prob::ODEProblem` and `sol::ODESolution`.
    - With a keyword argument `datum_format`, it results `prob`, `sol`, and `df::DataFrame`.
- `DatumFormat`: return a function `(x, t, integrator::DiffEqBase.DEIntegrator) -> nt::NamedTuple` for saving data.
- `save_inputs(func; kwargs...)`: this mimics `apply_inputs(func; kwargs...)`.
- `Process`: return a function that processes `prob` and `sol` to get simulation data.
- `save`: save `env`, `prob`, `sol`, and optionally `process`,
in a `.jld2` file.
- `load`: load `env`, `prob`, `sol`, and optionally `process`,
from a `.jld2` file.

## Usage
### Optimal Control and reinforcement learning
- For an example of **infinite-horizon continuous-time linear quadratic regulator (LQR)**, see the following example code (`test/lqr.jl`).

```julia
using FlightSims
const FS = FlightSims
using LinearAlgebra
using Plots


function test()
    # linear system
    n, m = 2, 1
    env = LinearSystemEnv(n, m)  # exported from FlightSims
    x0 = State(env)([1.0, 2.0])
    A = [0 1;
         0 0]
    B = [0;
         1]
    p = Params(env)(A, B)
    # optimal control
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    lqr = LQR(A, B, Q, R)  # exported from FlightSims
    u_lqr = FS.OptimalController(lqr)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
    # simulation
    t0, tf = 0.0, 10.0
    Δt = 0.01  # save period; not simulation time step
    # case 1: processing data simultaneously
    prob, sol, df = sim(
                        x0,  # initial condition
                        apply_inputs(Dynamics!(env); u=u_lqr),  # dynamics with input of LQR
                        p;
                        t0=t0, tf=tf,  # final time
                        datum_format=save_inputs(DatumFormat(env); input=u_lqr),  # saving data; default key: time, state
                        saveat=t0:Δt:tf,
                       )
    # case 2: processing data after simulation
    # df = Process(env)(prob, sol; Δt=0.01)  # DataFrame; `Δt` means data sampling period.
    plot(df.time, hcat(df.state...)'; title="state variable", label=["x1" "x2"])  # Plots
    savefig("figures/x_lqr.png")
    plot(df.time, hcat(df.input...)'; title="control input", label="u")  # Plots
    savefig("figures/u_lqr.png")
end
```

![ex_screenshot](./figures/x_lqr.png)
![ex_screenshot](./figures/u_lqr.png)

- For an example of **continuous-time value-iteration adaptive dynamic programming (CT-VI-ADP)**, take a look at `main/continuous_time_vi_adp.jl`.
    - [T. Bian and Z.-P. Jiang, “Value Iteration, Adaptive Dynamic Programming, and Optimal Control of Nonlinear Systems,” in 2016 IEEE 55th Conference on Decision and Control (CDC), Las Vegas, NV, USA, Dec. 2016, pp. 3375–3380. doi: 10.1109/CDC.2016.7798777.](https://ieeexplore.ieee.org/document/7798777)

### Nonlinear control
- For an example of **backstepping position tracking controller for quadcopters**,
see the following example code (`main/backstepping_tracking.jl`).
```julia
using FlightSims
const FS = FlightSims
using UnPack, ComponentArrays
using Transducers
using Plots


function make_env()
    multicopter = IslamQuadcopterEnv()
    @unpack m, g = multicopter
    x0_multicopter = State(multicopter)()
    pos0 = copy(x0_multicopter.p)
    vel0 = copy(x0_multicopter.v)
    helixCG = FS.HelixCommandGenerator(pos0)
    cg = command_generator(helixCG)
    controller = BacksteppingPositionControllerEnv(m; x_cmd_func=cg)
    x0_controller = State(controller)(pos0, m, g)
    x0 = ComponentArray(multicopter=x0_multicopter, controller=x0_controller)
    multicopter, controller, x0, cg
end

function main()
    multicopter, controller, x0, cg = make_env()
    prob, sol = sim(x0, Dynamics!(multicopter, controller); tf=40.0)
    df = Process()(prob, sol; Δt=0.01)
    ts = df.times
    poss = df.states |> Map(state -> state.multicopter.p) |> collect
    poss_true = ts |> Map(t -> cg(t)) |> collect
    ## plot
    # time vs position
    p_pos = plot(; title="position", legend=:outertopright)
    plot!(p_pos, ts, hcat(poss...)'; label=["x" "y" "z"], color="red", ls=[:dash :dot :dashdot])
    plot!(p_pos, ts, hcat(poss_true...)'; label=["x (true)" "y (true)" "z (true)"], color="black", ls=[:dash :dot :dashdot])
    savefig(p_pos, "t_vs_pos.png")
    # 3d traj
    p_traj = plot3d(; title="position", legend=:outertopright)
    plot!(p_traj, hcat(poss...)'[:, 1], hcat(poss...)'[:, 2], hcat(poss...)'[:, 3]; label="position", color="red")
    plot!(p_traj, hcat(poss_true...)'[:, 1], hcat(poss_true...)'[:, 2], hcat(poss_true...)'[:, 3]; label="position (true)", color="black")
    savefig(p_traj, "traj.png")
end
```
![ex_screenshot](./figures/backstepping_tracking.png)

### Scientific machine learning
- For an example usage of [Flux.jl](https://github.com/FluxML/Flux.jl), see `main/flux_example.jl`.
- For an example code of an imitation learning algorithm, behavioural cloning, see `main/behavioural_cloning.jl`.
