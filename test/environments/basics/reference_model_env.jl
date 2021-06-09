using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test()
    env = ReferenceModelEnv(4)
    X0_ref = State(env)(zeros(3))
    p = Params(env)()
    prob, sol = sim(X0_ref, apply_inputs(Dynamics!(env); x_cmd=[1, 2, 3]), p; tf=10.0)
    df = Process(env)(prob, sol)
    xs = df.state |> Map(X -> X.x_0) |> collect
    plot(df.time, hcat(xs...)')
end

function test_auto_diff()
    tf = 30
    _x_cmd_func = (t) -> [sin(t), cos(t), t]
    env_ad = ReferenceModelEnv(4; x_cmd_func=_x_cmd_func)
    env = ReferenceModelEnv(4)
    X0_ref = State(env)(zeros(3))
    p = Params(env)()
    prob_ad, sol_ad = sim(X0_ref, Dynamics!(env_ad), p; tf=tf)
    prob, sol = sim(X0_ref, apply_inputs(Dynamics!(env); x_cmd=(x, p, t) -> _x_cmd_func(t)), p; tf=tf)
    df_ad = Process(env)(prob_ad, sol_ad)
    df = Process(env)(prob, sol)
    p = plot(; legend=:outertopleft)
    xs = df.state |> Map(X -> X.x_0) |> collect
    xs_ad = df_ad.state |> Map(X -> X.x_0) |> collect
    xs_true = df.time |> Map(t -> _x_cmd_func(t)) |> collect
    plot!(p, df.time, hcat(xs...)'; label="naive tracking as set-point regulation", color="red")
    plot!(p, df_ad.time, hcat(xs_ad...)'; label="tracking with auto_diff", color="blue")
    plot!(p, df.time, hcat(xs_true...)'; label="true", color="black")
end
