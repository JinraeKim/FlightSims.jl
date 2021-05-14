# TwoDimensionalNonlinearPolynomialEnv
struct TwoDimensionalNonlinearPolynomialEnv <: AbstractEnv
end

function State(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x1::Real=-2.9, x2::Real=-2.9)
        ComponentArray(x1=x1, x2=x2)
    end
end

function dynamics!(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (dx, x, p, t; u)
        @unpack x1, x2 = x
        dx.x1 = -(1/2)*x1^3 - x1 - 2*x2
        dx.x2 = (1/8)*x2^3 - x2 + (1/2)*u^3
        nothing
    end
end

function optimal_input(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x::ComponentArray, p, t)
        @unpack x1, x2 = x
        -x2
    end
end

function optimal_value(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x::ComponentArray)
        @unpack x1, x2 = x
        x1^2 + x2^2
    end
end

function running_cost(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x::ComponentArray, _u)
        @assert length(_u) == 1
        u = _u[1]  # Real or Array
        @unpack x1, x2 = x
        x1^4 + 2*(x1+x2)^2 + (3/4)*u^4
    end
end

