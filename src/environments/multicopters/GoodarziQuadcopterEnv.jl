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
"""
Base.@kwdef struct GoodarziQuadcopterEnv <: QuadcopterEnv
    m = 0.5  # kg
    J = 1e-2 * Diagonal([0.557, 0.557, 1.050])  # kg m²
    J_inv = inv(J)
    g = 9.81  # m/s²
end

# """
# # Variables
# f ∈ R: thrust magnitude
# M ∈ R^3: moment
# """
# function dynamics!(env::GoodarziQuadcopterEnv)
#     return function (dX, X, p, t; f, M)
#         _dynamics!(env)(dX, X, p, t; f=f, M=M)
#     end
# end
