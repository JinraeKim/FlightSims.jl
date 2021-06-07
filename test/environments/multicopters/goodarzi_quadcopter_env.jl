using FlightSims
using UnPack
using Plots
using Transducers
using Rotations: RotXYZ
using Test


@testset "GoodarziQuadcopterEnv" begin
    dir_log = "data/test/environments/multicopters/GoodarziQuadcopterEnv"
    mkpath(dir_log)

    env = GoodarziQuadcopterEnv()
    state0 = State(env)()
    state0.ω += [0.01, 0, -0.1]
    @unpack m, g = env
    @time prob, sol = sim(state0, apply_inputs(dynamics!(env); u=[m*g, zeros(3)...]); tf=10.0)
    ts = prob.tspan[1]:0.01:prob.tspan[end]
    data = ts |> Map(t -> sol(t)) |> collect
    for sym in [:p, :v, :R, :ω]
        _data = data |> Map(datum -> getproperty(datum, sym)) |> collect
        if sym == :R
            _data = _data |> Map(R -> euler(RotXYZ(R))) |> tcollect
        end
        _sym = sym == :R ? :euler : sym
        p = plot(ts, hcat(_data...)'; label=String(_sym))
        savefig(p, joinpath(dir_log, String(_sym) * ".png"))
    end
    println("Test results are saved in $(dir_log)")
end
