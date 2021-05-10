### Types
abstract type AbstractEnv end

### Envs
## TwoDimensionalNonlinearPolynomialEnv
struct TwoDimensionalNonlinearPolynomialEnv <: AbstractEnv
end

function State(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x1::Real=-2.9, x2::Real=-2.9)
        ComponentArray(x1=x1, x2=x2)
    end
end

function dynamics(env::TwoDimensionalNonlinearPolynomialEnv)
    f = ẋ(env)
    return (x, p, t) -> f(x, optimal_input(env)(x))  # default: optimal control
end
function dynamics!(env::TwoDimensionalNonlinearPolynomialEnv)
    f = ẋ(env)
    return function (dx, x, p, t)
        @unpack x1, x2 = x
        u = optimal_input(env)(x)  # default: optimal control
        dx.x1 = ẋ1(env)(x1, x2)
        dx.x2 = ẋ2(env)(x2, u)
    end
    return nothing
end

ẋ1(env::TwoDimensionalNonlinearPolynomialEnv) = (x1, x2) -> -(1/2)*x1^3 - x1 - 2*x2
ẋ2(env::TwoDimensionalNonlinearPolynomialEnv) = (x2, u) -> (1/8)*x2^3 - x2 + (1/2)*u^3
function ẋ(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x, u::Real)
        @unpack x1, x2 = x
        dx1 = ẋ1(env)(x1, x2)
        dx2 = ẋ2(env)(x2, u)
        State(env)(dx1, dx2)
    end
end

function optimal_input(env::TwoDimensionalNonlinearPolynomialEnv)
    return function (x::ComponentArray)
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
    return function (x::ComponentArray, u::Real)
        @unpack x1, x2 = x
        x1^4 + 2*(x1+x2)^2 + (3/4)*u^4
    end
end
