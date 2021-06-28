using FlightSims
using ComponentArrays
using UnPack
using Transducers
using Plots


function test()
    K_P, K_I, K_D = 1e1, 1e0, 1e0
    controller = PID(K_P, K_I, K_D)
    x0_pid = State(controller)()
    A = ones(1, 1)
    B = ones(1, 1)
    linear_system = LinearSystemEnv(A, B)
    x0 = State(linear_system)([1])
    X0 = ComponentArray(x=x0, pid=x0_pid)
    command_pid = Command(controller)
    x_des = zeros(1)
    tf = 20.0
    @Loggable function dynamics!(dX, X, p, t)
        @unpack ∫e, ê = X.pid
        @unpack x = X
        e = x - x_des
        u = command_pid(e, ∫e, ê)
        @nested_log Dynamics!(linear_system)(dX.x, X.x, (), t; u=u)
        @nested_log Dynamics!(controller)(dX.pid, X.pid, (), t; e=e)
    end
    prob, df = sim(X0,
                   dynamics!;
                   tf=tf,
                  )
    df
    # plot
    ts = df.time
    es = df.sol |> Map(datum -> datum.e) |> collect
    ês = df.sol |> Map(datum -> datum.ê) |> collect
    p = plot(; title="error")
    plot!(p, ts, hcat(es...)'; label="e")
    plot!(p, ts, hcat(ês...)'; label="ê")
end
