### Types
abstract type AbstractEnv end

### Envs
## Basic environments
# TwoDimensionalNonlinearPolynomialEnv
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
    return function (dx, x, p, t)
        @unpack x1, x2 = x
        u = optimal_input(env)(x)  # default: optimal control
        dx.x1 = ẋ1(env)(x1, x2)
        dx.x2 = ẋ2(env)(x2, u)
        nothing
    end
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

## Multicopters
abstract type Multicopter <: AbstractEnv end
abstract type Quadcopter <: Multicopter end
# Quadcopter
"""
# References
[1] F. A. Goodarzi, D. Lee, and T. Lee,
“Geometric stabilization of a quadrotor UAV with a payload connected by flexible cable,”
in 2014 American Control Conference, Jun. 2014, pp. 4925–4930, doi: 10.1109/ACC.2014.6859419.
# Variables
p ∈ R^3: inertial position
v ∈ R^3: inertial velocity
R ∈ so(3): body-to-inertial rotation matrix (R_B2I)
ω ∈ R^3: angular rate of body in inertial frame
"""
struct GoodarziQuadcopter <: Quadcopter
    m
    J
    J_inv
    g
end
function GoodarziQuadcopter(;
        m = 0.5,  # kg
        J = 1e-2 * Diagonal([0.557, 0.557, 1.050]),  # kg m^2
        J_inv = inv(J),
        g = 9.81,  # m/s^2
    )
    GoodarziQuadcopter(m, J, J_inv, g)
end

function State(env::GoodarziQuadcopter)
    return function (p=zeros(3), v=zeros(3),
                     R=one(RotMatrix{3}),
                     ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
    end
end

"""
f ∈ R: thrust magnitude
M ∈ R^3: moment
"""
function ẋ(env::GoodarziQuadcopter)
    @unpack m, g, J, J_inv = env
    e3 = [0, 0, 1]
    skew(x) = [    0 -x[3]  x[2];
                x[3]     0 -x[1];
               -x[2]  x[1]    0]
    return function (state, f, M)
        @unpack p, v, R, ω = state
        ṗ = v
        v̇ = -(1/m)*f*R*e3 + g*e3
        Ω = skew(ω)
        Ṙ = R*Ω
        ω̇ = J_inv * (-Ω*J*ω + M)
        ṗ, v̇, Ṙ, ω̇
    end
end

function dynamics(env::GoodarziQuadcopter)
    deriv = ẋ(env)
    @unpack m, g = env
    f, M = m*g, zeros(3)  # default input
    (state, p, t) -> State(env)(deriv(state, f, M)...)
end

function dynamics!(env::GoodarziQuadcopter)
    deriv = ẋ(env)
    @unpack m, g = env
    return function (dstate, state, p, t)
        @unpack p, v, R, ω = state
        f, M = m*g, zeros(3)  # default input
        ṗ, v̇, Ṙ, ω̇ = deriv(state, f, M)
        dstate.p = ṗ
        dstate.v = v̇
        dstate.R = Ṙ
        dstate.ω = ω̇
        nothing
    end
end
