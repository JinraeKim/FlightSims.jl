# QuadcopterEnv
"""
# References
[1] F. A. Goodarzi, D. Lee, and T. Lee,
“Geometric stabilization of a quadrotor UAV with a payload connected by flexible cable,”
in 2014 American Control Conference, Jun. 2014, pp. 4925–4930, doi: 10.1109/ACC.2014.6859419.
# Variables
p ∈ R^3: inertial position
v ∈ R^3: inertial velocity
R ∈ so(3): inertial-to-body rotation matrix (R_I2B)
ω ∈ R^3: angular rate of body in inertial frame
u ∈ R^4: [f, M...] where f ∈ R: total thrust, M ∈ R^3: moment
"""
Base.@kwdef struct GoodarziQuadcopterEnv <: QuadcopterEnv
    m = 0.5  # kg
    J = 1e-2 * Diagonal([0.557, 0.557, 1.050])  # kg m²
    J_inv = inv(J)
    g = 9.81  # m/s²
    # input limits
    dim_input = 4
    u_min = zeros(dim_input)
    u_max = (m*g) * ones(dim_input)
end

function saturate(env::GoodarziQuadcopterEnv, u)
    @unpack u_min, u_max = env
    u_saturated = clamp.(u, u_min, u_max)
end

function input_to_force_moment(env::GoodarziQuadcopterEnv, u)
    ν = u
end
