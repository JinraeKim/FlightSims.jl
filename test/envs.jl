using FlightSims
using ComponentArrays
using OrdinaryDiffEq
using UnPack
using Plots
using Transducers
using Rotations: RotXYZ


function test_quadcopter(dir_log)
    mkpath(dir_log)
    env = GoodarziQuadcopter()
    state0 = State(env)()
    state0.ω += [0.01, 0, -0.1]
    @unpack m, g = env
    @time prob, sol = sim(env, state0, apply_inputs(dynamics!(env); f=m*g, M=zeros(3)); tf=10.0)
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
end

function test_twodimnonlinearpoly(dir_log)
    mkpath(dir_log)
    env = TwoDimensionalNonlinearPolynomialEnv()
    x0 = State(env)(-2, 3)
    @time prob, sol = sim(env, x0, apply_inputs(dynamics!(env); u=FlightSims.optimal_input(env)); tf=10.0)
    p = plot(sol)
    savefig(p, joinpath(dir_log, "x.png"))
end


function test()
    dir_log = "data"
    test_twodimnonlinearpoly(joinpath(dir_log, "twodimnonlinearpoly"))
    test_quadcopter(joinpath(dir_log, "quadcopter"))
end
