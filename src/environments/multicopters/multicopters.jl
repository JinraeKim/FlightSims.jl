# multicopter
abstract type MulticopterEnv <: AbstractEnv end

"""
Common state structure of MulticopterEnv
"""
function State(env::MulticopterEnv)
    return function (p=zeros(3), v=zeros(3), R=one(RotMatrix{3}), ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
    end
end

"""
Common input saturation (default: not applied)
"""
function saturate(env::MulticopterEnv, u)
    u
end

"""
Common input to force and moment transformation (default: not applied)
"""
function input_to_force_moment(env::MulticopterEnv, u)
    ν = u
end

function __Dynamics!(env::MulticopterEnv)
    @unpack m, g, J = env
    J_inv = inv(J)
    e3 = [0, 0, 1]
    skew(x) = [    0 -x[3]  x[2];
                x[3]     0 -x[1];
               -x[2]  x[1]    0]
    return function (dX, X, p, t; f, M)
        @unpack p, v, R, ω = X
        Ω = skew(ω)
        dX.p = v
        dX.v = -(1/m)*f*R'*e3 + g*e3
        dX.R = -Ω*R
        dX.ω = J_inv * (-Ω*J*ω + M)
        nothing
    end
end

"""
Common dynamics of MulticopterEnv.
# Variables
f ∈ R: total thrust
M ∈ R^3: moment
"""
function Dynamics!(env::MulticopterEnv)
    return function (dX, X, p, t; u)
        u_saturated = saturate(env, u)
        ν = input_to_force_moment(env, u_saturated)
        f, M = ν[1], ν[2:4]
        __Dynamics!(env)(dX, X, p, t; f=f, M=M)
    end
end

# """
# Common dynamics of MulticopterEnv.
# # Variables
# f ∈ R: total thrust
# M ∈ R^3: moment
# """
# function Dynamics!(env::MulticopterEnv;
#         faults::Vector{T} where T <: AbstractFault=AbstractFault[],
#     )
#     actuator_faults = faults |> Filter(fault -> typeof(fault) <: AbstractActuatorFault) |> collect
#     actuator_faults_groups = SplitApplyCombine.group(fault -> fault.index, actuator_faults)  # actuator_faults groups classified by fault index
#     return function (dX, X, p, t; u)
#         u_saturated = saturate(env, u)
#         _u_faulted = u_saturated
#         # actuator faults
#         for actuator_faults_group in actuator_faults_groups
#             last_actuator_fault = select_last_before_t(actuator_faults_group, t)
#             _u_faulted = last_actuator_fault(t, _u_faulted)
#         end
#         u_faulted = _u_faulted
#         # @show u_faulted ./ u_saturated, t  # for debugging
#         ν = input_to_force_moment(env, u_faulted)
#         f, M = ν[1], ν[2:4]
#         __Dynamics!(env)(dX, X, p, t; f=f, M=M)
#     end
# end

# function process(env::MulticopterEnv)
#     return function (prob::ODEProblem, sol::ODESolution; Δt=0.01)
#         t0, tf = prob.tspan
#         ts = t0:Δt:tf
#         xs = ts |> Map(t -> sol(t)) |> collect
#         ps = xs |> Map(x -> x.p) |> collect
#         vs = xs |> Map(x -> x.v) |> collect
#         Rs = xs |> Map(x -> x.R) |> collect
#         ωs = xs |> Map(x -> x.ω) |> collect
#         DataFrame(
#                   times=ts,
#                   states=xs,
#                   positions=ps,
#                   velocities=vs,
#                   rotation_matricies=Rs,
#                   angular_rates=ωs,
#                  )
#     end
# end


# multicopters
include("quadcopters.jl")
include("hexacopters.jl")
