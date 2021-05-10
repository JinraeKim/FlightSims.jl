using FlightSims
using ComponentArrays
using OrdinaryDiffEq
using Parameters: @unpack
using Plots
using Transducers
using Rotations: RotXYZ


function test_quadcopter(dir_log)
    mkpath(dir_log)
    env = GoodarziQuadcopter()
    state0 = State(env)()
    state0.ω += [0.01, 0, -0.1]
    @time prob, sol = sim(env, state0; tf=10.0)
    ts = prob.tspan[1]:0.01:prob.tspan[end]
    for sym in [:p, :v, :R, :ω]
        data = ts |> Map(t -> getproperty(sol(t), sym)) |> collect
        if sym == :R
            data = data |> Map(R -> euler(RotXYZ(R))) |> collect
        end
        _sym = sym == :R ? :euler : sym
        p = plot(ts, hcat(data...)'; label=String(_sym))
        savefig(p, joinpath(dir_log, String(_sym) * ".png"))
    end
end

function test_twodimnonlinearpoly(dir_log)
    mkpath(dir_log)
    env = TwoDimensionalNonlinearPolynomialEnv()
    x0 = State(env)(-2, 3)
    @time prob, sol = sim(env, x0; tf=10.0)
    p = plot(sol)
    savefig(p, joinpath(dir_log, "x.png"))
end


function test()
    dir_log = "data"
    test_twodimnonlinearpoly(joinpath(dir_log, "twodimnonlinearpoly"))
    test_quadcopter(joinpath(dir_log, "quadcopter"))
end
