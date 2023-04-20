using FlightSims
import FlightSims: State, Dynamics!
using DataFrames
using ComponentArrays
using Transducers
using Plots
using SciMLBase
using Test


struct LinearSystem_ZOH_Input <: AbstractEnv
    linear_env::LinearSystem
end

function State(env::LinearSystem_ZOH_Input)
    State(env.linear_env)
end

function Dynamics!(env::LinearSystem_ZOH_Input)
    @Loggable function dynamics!(dx, x, input, t)
        @nested_log Dynamics!(env.linear_env)(dx, x, nothing, t; u=input)
    end
end

function main()
    A = [0 1;
         0 0]
    B = [0 1]'
    linear_env = LinearSystem(A, B)
    env = LinearSystem_ZOH_Input(linear_env)
    x0_state = [1, 2]
    input = zeros(1)
    x0 = State(env)(x0_state)
    t0 = 0.0
    tf = 20.0
    simulator = Simulator(x0, Dynamics!(env), input; tf=tf)
    # interactive sim
    Δt = 0.1  # save period
    df = DataFrame()
    @time for (i, t) in enumerate(t0:Δt:tf)
        # To perform interactive simulation,
        # you should be aware of the integrator interface
        # provided by DifferentialEquations.jl;
        # see https://diffeq.sciml.ai/stable/basics/integrator/#integrator
        # e.g., DO NOT directly change the integrator state
        state = simulator.integrator.u
        input = simulator.integrator.p
        if (i-1) % 10 == 0  # update input period
            input .= -sum(state)
        end
        step_until!(simulator, t, df)
    end
    # plot
    ts = df.time
    states = df.sol |> Map(datum -> datum.state) |> collect
    inputs = df.sol |> Map(datum -> datum.input) |> collect
    p_x = plot(ts, hcat(states...)';
               title="state variable", label=["x1" "x2"],
              )
    p_u = plot(ts, hcat(inputs...)';
               title="input variable", label="u",
               linetype=:steppost,  # to plot zero-order-hold input appropriately
              )
    fig = plot(p_x, p_u; layout=(2, 1))
    savefig("figures/interactive_sim.png")
    display(fig)
end


@testset "linear_system_zoh_input" begin
    main()
end
