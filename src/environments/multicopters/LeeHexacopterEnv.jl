"""
# Notes
- [1] is modified for hexa-x configuration; see [2].
# References
[1] T. Lee, M. Leok, and N. H. McClamroch,
“Geometric Tracking Control of a Quadrotor UAV on SE(3),”
in 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, Dec. 2010, pp. 5420–5425. doi: 10.1109/CDC.2010.5717652.
[2] PX4 Airframes Reference, https://docs.px4.io/master/en/airframes/airframe_reference.html.
# Variables
u ∈ R^6: rotor forces
"""
Base.@kwdef struct LeeHexacopterEnv <: HexaCopterEnv
    J = Diagonal([0.0820, 0.0845, 0.1377])  # kg m^2
    l = 0.315  # m
    kM = 8.004e-4  # m s^2
    m = 4.34  # kg
    g = 9.81  # m / s^2
    B = [ 1   1             1              1             1              1;
         -l   l         0.5*l         -0.5*l        -0.5*l          0.5*l;
          0   0 0.5*sqrt(3)*l -0.5*sqrt(3)*l 0.5*sqrt(3)*l -0.5*sqrt(3)*l;
         kM -kM            kM            -kM           -kM             kM]
    u_min = zeros(6)
    u_max = (m*g) * ones(6)
end

function saturate(env::LeeHexacopterEnv, u)
    @unpack u_min, u_max = env
    u_saturated = clamp.(u, u_min, u_max)
end

function input_to_force_moment(env::LeeHexacopterEnv, u)
    @unpack B = env
    ν = B * u
end
