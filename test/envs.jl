using FlightSims
using ComponentArrays
using DifferentialEquations
using Parameters: @unpack
using Plots


function test()
    tspan = (0.0, 10.0)
    env = TwoDimensionalNonlinearPolynomialEnv()
    x0 = State(env)(-2, 3)
    prob = ODEProblem(dynamics!(env), x0, tspan)
    @time sol = solve(prob, Tsit5())
    plot(sol)
end

function test_nested()
    tspan = (0.0, 100.0)
    x0 = ComponentArray(x1=1.0, x2=2.0)
    y0 = ComponentArray(y1=1.0, y2=2.0)
    X0 = ComponentArray(x=x0, y=y0)
    dyn! = function (D, X, p, t)
        @unpack x, y = X
        @unpack x1, x2 = x
        @unpack y1, y2 = y
        D.x.x1 = -x2
        D.x.x2 = -x1
        D.y = -y
    end
    prob = ODEProblem(dyn!, X0, tspan)
    # dyn = function (X, p, t)
    #     @unpack x, y = X
    #     @unpack x1, x2 = x
    #     @unpack y1, y2 = y
    #     dX = ComponentArray(x=(x1=-x2, x2=-x1), y=(y1=y1, y2=y2))
    # end
    # prob = ODEProblem(dyn, X0, tspan)
    @time sol = solve(prob, Tsit5())
end
