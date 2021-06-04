using FlightSims
const FS = FlightSims
using Transducers
using Plots


function test()
    env = ReferenceModelEnv(4)
    x0 = zeros(3)
    X0_ref = State(env)(x0)
    prob, sol = sim(X0_ref, apply_inputs(dynamics!(env); x_cmd=[1, 2, 3]); tf=10.0)
    df = process(env)(prob, sol)
    xs = df.states |> Map(X -> X.x_0) |> collect
    plot(df.times, hcat(xs...)')
end

function test_auto_diff()
    tf = 30
    _x_cmd_func = (t) -> [sin(t), cos(t), t]
    env_ad = ReferenceModelEnv(4; x_cmd_func=_x_cmd_func)
    env = ReferenceModelEnv(4)
    x0 = zeros(3)
    X0_ref = State(env)(x0)
    prob_ad, sol_ad = sim(X0_ref, dynamics!(env_ad); tf=tf)
    prob, sol = sim(X0_ref, apply_inputs(dynamics!(env); x_cmd=(x, p, t) -> _x_cmd_func(t)); tf=tf)
    df_ad = process(env)(prob_ad, sol_ad)
    df = process(env)(prob, sol)
    p = plot(; legend=:outertopleft)
    xs = df.states |> Map(X -> X.x_0) |> collect
    xs_ad = df_ad.states |> Map(X -> X.x_0) |> collect
    xs_true = df.times |> Map(t -> _x_cmd_func(t)) |> collect
    plot!(p, df.times, hcat(xs...)'; label="naive tracking as set-point regulation", color="red")
    plot!(p, df_ad.times, hcat(xs_ad...)'; label="tracking with auto_diff", color="blue")
    plot!(p, df.times, hcat(xs_true...)'; label="true", color="black")
end
