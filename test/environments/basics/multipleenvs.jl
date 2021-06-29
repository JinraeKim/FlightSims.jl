using FlightSims
using Transducers
using Plots
using ComponentArrays


function test()
    pid_keys = (:pid1, :pid2, :pid3)
    K_P, K_I, K_D = 1e1, 1e0, 1e0
    pids = pid_keys |> Map(key -> PID(K_P, K_I, K_D)) |> collect
    controller = MultipleEnvs(Dict(key => value for (key, value) in zip(pid_keys, pids)))
    x0 = State(controller)(;
                           pid1=State(controller.envs_dict[:pid1])(),
                           pid2=State(controller.envs_dict[:pid2])(),
                           pid3=State(controller.envs_dict[:pid3])(),
                          )
    tf = 5.0
    prob, df = sim(x0,
                   apply_inputs(Dynamics!(controller);
                                pid1=Dict(:e => [1.0]),
                                pid2=Dict(:e => [0.0]),
                                pid3=Dict(:e => [-1.0]),
                               );
                   tf=tf)
    @show df
    sol_pid1 = df.sol |> Map(datum -> datum.pid1.∫e) |> collect
    sol_pid2 = df.sol |> Map(datum -> datum.pid2.∫e) |> collect
    sol_pid3 = df.sol |> Map(datum -> datum.pid3.∫e) |> collect
    p_pid1 = plot(df.time, hcat(sol_pid1...)'; title="pid1")
    p_pid2 = plot(df.time, hcat(sol_pid2...)'; title="pid2")
    p_pid3 = plot(df.time, hcat(sol_pid3...)'; title="pid3")
    plot(p_pid1, p_pid2, p_pid3; layout=(3, 1))
end
