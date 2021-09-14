using FlightSims
const FS = FlightSims

using LinearAlgebra  # for I, e.g., Matrix(I, n, n)
using ComponentArrays
using UnPack
using Transducers
using Plots
using DifferentialEquations  # for callbacks


struct MyEnv <: AbstractEnv  # AbstractEnv exported from FS
    a
    b
end

"""
FlightSims recommends you to use closures for State and Dynamics!. For more details, see https://docs.julialang.org/en/v1/devdocs/functions/.
"""
function State(env::MyEnv)
    return function (x1::Number, x2::Number)
        ComponentArray(x1=x1, x2=x2)
    end
end

function Dynamics!(env::MyEnv)
    @unpack a, b = env  # @unpack is very useful!
    @Loggable function dynamics!(dx, x, p, t; u)  # `Loggable` makes it loggable via SimulationLogger.jl (imported in FS)
        @unpack x1, x2 = x
        @log x1  # to log x1
        @log x2  # to log x2
        dx.x1 = a*x2
        dx.x2 = b*u
    end
end

function my_controller(x, p, t)
    @unpack x1, x2 = x
    -(x1+x2)
end


function main()
    n = 2
    m = 1
    a, b = 1, 1
    A = -Matrix(I, n, n)
    B = Matrix(I, m, m)
    env = MyEnv(a, b)
    tf = 10.0
    Δt = 0.01
    # callbacks
    function condition(u, t, integrator)  # DiffEq.jl convention
        @unpack x1, x2 = u
        x1 - 0  # i.e., it stops when x1 = 0
    end
    affect!(integrator) = terminate!(integrator)  # See DiffEq.jl documentation
    cb_diverge = ContinuousCallback(condition, affect!)
    cb = CallbackSet(cb_diverge,)  # useful for multiple callbacks
    x10, x20 = 10.0, 0.0
    x0 = State(env)(x10, x20)
    # prob: DE problem, df: DataFrame
    @time prob, df = sim(
                         x0,  # initial condition
                         apply_inputs(Dynamics!(env); u=my_controller);  # dynamics!; apply_inputs is exported from FS and is so useful for systems with inputs
                         tf=10.0,
                         savestep=Δt,  # savestep is NOT simulation step
                         callback=cb,
                        )  # sim is exported from FS
    ts = df.time
    x1s = df.sol |> Map(datum -> datum.x1) |> collect
    x2s = df.sol |> Map(datum -> datum.x2) |> collect
    # plot
    p_x1 = plot(ts, x1s;
                label="x1",
               )
    p_x2 = plot(ts, x2s;
                label="x2",
               )
    p_x = plot(p_x1, p_x2, layout=(2, 1))
    # save
    dir_log = "figures"
    mkpath(dir_log)
    savefig(p_x, joinpath(dir_log, "toy_example.png"))
    display(p_x)
end
