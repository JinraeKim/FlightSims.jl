"""
# References
[1] M. Islam, M. Okasha, and M. M. Idres, “Dynamics and control of quadcopter using linear model predictive control approach,” IOP Conf. Ser.: Mater. Sci. Eng., vol. 270, p. 012007, Dec. 2017, doi: 10.1088/1757-899X/270/1/012007.
"""
Base.@kwdef struct IslamQuadcopterEnv <: QuadcopterEnv
    J = Diagonal([7.5e-3, 7.5e-3, 1.3e-2])  # kg m^2
    l = 0.23  # m
    kf = 3.13e-5  # N s^2
    kM = 7.5e-7  # N m s^2
    m = 0.65  # kg
    g = 9.81  # m / s^2
    B = [kf  kf  kf  kf;
          0 -kf   0  kf;
         kf   0 -kf   0;
         kM -kM  kM -kM]
    # drags
    # kt = Diagonal(0.1*ones(3))  # Ns / m
    # kr = Diagonal(0.1*ones(3))  # N m s
    # input limits
    u_min = zeros(4)
    u_max = (m*g/kf) * ones(4)
end

function saturate(env::IslamQuadcopterEnv, u)
    @unpack u_min, u_max = env
    u_saturated = clamp.(u, u_min, u_max)
end

function input_to_force_moment(env::IslamQuadcopterEnv, u)
    @unpack B = env
    ν = B * u
end

# function dynamics!(env::IslamQuadcopterEnv)
#     @unpack B = env
#     return function (dX, X, p, t; u)
#         _u = saturate(env, u)
#         ν = B * _u
#         f, M = ν[1], ν[2:4]
#         _dynamics!(env)(dX, X, p, t; f=f, M=M)
#     end
# end
