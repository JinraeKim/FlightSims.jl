# QuadcopterEnv
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
Base.@kwdef struct GoodarziQuadcopterEnv <: QuadcopterEnv
    m = 0.5  # kg
    J = 1e-2 * Diagonal([0.557, 0.557, 1.050])  # kg m²
    J_inv = inv(J)
    g = 9.81  # m/s²
end

function State(env::GoodarziQuadcopterEnv)
    return function (p=zeros(3), v=zeros(3),
                     R=one(RotMatrix{3}),
                     ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
    end
end

"""
# Variables
f ∈ R: thrust magnitude
M ∈ R^3: moment
"""
function dynamics!(env::GoodarziQuadcopterEnv)
    @unpack m, g, J, J_inv = env
    e3 = [0, 0, 1]
    skew(x) = [    0 -x[3]  x[2];
                x[3]     0 -x[1];
               -x[2]  x[1]    0]
    return function (dstate, state, p, t; f=f, M=M)
        @unpack p, v, R, ω = state
        Ω = skew(ω)
        dstate.p = v
        dstate.v = -(1/m)*f*R*e3 + g*e3
        dstate.R = R*Ω
        dstate.ω = J_inv * (-Ω*J*ω + M)
        nothing
    end
end
