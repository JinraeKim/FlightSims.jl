abstract type QuadcopterEnv <: MulticopterEnv end

"""
Common state structure of QuadcopterEnv
"""
function State(env::QuadcopterEnv)
    return function (p=zeros(3), v=zeros(3), R=one(RotMatrix{3}), ω=zeros(3))
        ComponentArray(p=p, v=v, R=R, ω=ω)
    end
end

"""
Common input saturation (default: not applied)
"""
function saturate(env::QuadcopterEnv, u)
    u
end

"""
Common input to force and moment transformation (default: not applied)
"""
function input_to_force_moment(env::QuadcopterEnv, u)
    ν = u
end

"""
Common dynamics of QuadcopterEnv.
# Variables
f ∈ R: total thrust
M ∈ R^3: moment
"""
function dynamics!(env::QuadcopterEnv; faults::Vector{T} where T <: AbstractFault=AbstractFault[])
    actuator_faults = faults |> Filter(fault -> typeof(fault) <: AbstractActuatorFault) |> collect
    actuator_faults_groups = SplitApplyCombine.group(fault -> fault.index, actuator_faults)  # actuator_faults groups classified by fault index
    return function (dX, X, p, t; u)
        u_saturated = saturate(env, u)
        _u_faulted = u_saturated
        for actuator_faults_group in actuator_faults_groups
            last_actuator_fault = select_last_before_t(actuator_faults_group, t)
            _u_faulted = last_actuator_fault(t, _u_faulted)
        end
        u_faulted = _u_faulted
        # @show u_faulted ./ u_saturated  # for debugging
        ν = input_to_force_moment(env, u_faulted)
        f, M = ν[1], ν[2:4]
        _dynamics!(env)(dX, X, p, t; f=f, M=M)
    end
end
function _dynamics!(env::QuadcopterEnv)
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

include("GoodarziQuadcopterEnv.jl")
include("IslamQuadcopterEnv.jl")
