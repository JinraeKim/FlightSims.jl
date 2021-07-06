# multicopter
abstract type MulticopterEnv <: AbstractEnv end

"""
Common state structure of MulticopterEnv
"""
function State(multicopter::MulticopterEnv)
    return function (p=zeros(3), v=zeros(3), R=DCM(I), ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
    end
end

"""
Common input saturation (default: not applied)
"""
function saturate(multicopter::MulticopterEnv, u)
    # u
    error("Actuator saturation not applied: `saturate`")
end

"""
Common input to force and moment transformation (default: not applied)
"""
function input_to_force_moment(multicopter::MulticopterEnv, u)
    # ν = u
    error("Transformation of input to force and moment not defined: `input_to_force_moment`")
end


"""
# Variables
## State
p ∈ R^3: (inertial) position
v ∈ R^3: (inertial) velocity
R ∈ so(3): rotation matrix of body frame w.r.t. inertial frame (I to B)
ω ∈ R^3: angular rate of body frame w.r.t. inertial frame (I to B)
## (Virtual) input
f ∈ R: total thrust
M ∈ R^3: moment
"""
function __Dynamics!(multicopter::MulticopterEnv)
    @unpack m, g, J = multicopter
    J_inv = inv(J)
    e3 = [0, 0, 1]
    skew(x) = [    0 -x[3]  x[2];
                x[3]     0 -x[1];
               -x[2]  x[1]    0]
    @Loggable function dynamics!(dX, X, p, t; f, M)
        @unpack p, v, R, ω = X
        @onlylog state = X
        @nested_log :input f, M
        Ω = skew(ω)
        dX.p = v
        dX.v = -(1/m)*f*R'*e3 + g*e3
        dX.R = -Ω*R
        dX.ω = J_inv * (-Ω*J*ω + M)
    end
end

# the following closure is a basic template when using multicopter envs in other packages
# function Dynamics!(multicopter::MulticopterEnv)
#     @Loggable function dynamics!(dX, X, p, t; u)
#         u_saturated = FlightSims.saturate(multicopter, u)
#         ν = FlightSims.input_to_force_moment(multicopter, u_saturated)
#         f, M = ν[1], ν[2:4]
#         @nested_log FlightSims.__Dynamics!(multicopter)(dX, X, p, t; f=f, M=M)
#     end
# end


# multicopters
include("quadcopters.jl")
include("hexacopters.jl")
