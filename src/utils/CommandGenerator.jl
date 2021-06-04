abstract type AbstractCommandGenerator end
abstract type AbstractMulticopterCommandGenerator <: AbstractCommandGenerator end

function command_generator(cg::AbstractCommandGenerator)
    error("Not implemented")
end

"""
# Refs.
[1] E. Kaufmann, A. Loquercio, R. Ranftl, M. Müller, V. Koltun, and D. Scaramuzza, “Deep Drone Acrobatics,” arXiv:2006.05768 [cs], Jun. 2020, Accessed: Jun. 02, 2021. [Online]. Available: http://arxiv.org/abs/2006.05768
# Variables
d ∈ R^3: displacement
r ∈ R^+: radius
# Notes
units: [m], [s]
"""
struct PowerLoop <: AbstractMulticopterCommandGenerator
    p0::Vector
    v0::Vector
    Δp::Vector
    r::Real
    t0::Real
    t_go_straight::Real
    t_loop::Real
    a::Real  # accel.
    ΔV::Real  # speed increment
end

function PowerLoop(;
        p0=zeros(3), v0=[1, 0, 0], d=4, r=1.5,
        t0=0.0, t_go_straight=1.0,)
    @assert r > 0
    @assert t_go_straight > 0
    Δp = d * (v0/norm(v0))
    Δp_norm = norm(Δp)
    v0_norm = norm(v0)
    a = (Δp_norm - v0_norm*t_go_straight) / (0.5*t_go_straight^2)
    ΔV = a*t_go_straight
    t_loop = (2*π*r) / (v0_norm+ΔV)
    PowerLoop(p0, v0, Δp, r, t0, t_go_straight, t_loop, a, ΔV)
end

"""
# Notes
NED coordinate system.
- go straight
norm(v0)*(t-t0) + 0.5*a*(t-t0)^2 = norm(Δp)
- loop
r*θ = V*(t-t0-t_go_straight)
"""
function command_generator(cg::PowerLoop; coordinate=:NED)
    @unpack t0, p0, v0, Δp, t_go_straight, r, a, ΔV, t_loop = cg
    Δp_norm = norm(Δp)
    v0_norm = norm(v0)
    Δp_unit = Δp / Δp_norm
    if coordinate == :NED
        down_unit = [0, 0, 1]
    elseif coordinate == :ENU
        down_unit = [0, 0, -1]
    end
    to_the_centre = cross(cross(down_unit, v0), v0)
    to_the_centre_unit = to_the_centre / norm(to_the_centre)  # unit vec
    v0_unit = v0 / v0_norm
    return function (t)
        p_cmd = nothing
        if t < t0
            # before start
            p_cmd = p0
        elseif t < t0 + t_go_straight
            # go straight
            _t0 = t0
            _p0 = p0
            p_cmd = _p0 + (v0_norm*(t-_t0) + 0.5*a*(t-_t0)^2) * Δp_unit
        elseif t < t0 + t_go_straight + t_loop
            # loop
            _t0 = t0 + t_go_straight
            _p0 = p0 + Δp + r*to_the_centre_unit
            θ = (v0_norm+ΔV)*(t-_t0) / r
            p_cmd = _p0 - r*to_the_centre_unit*cos(θ) + v0_unit*sin(θ)
        elseif t < t0 + t_go_straight + t_loop + t_go_straight
            # go straight
            _t0 = t0 + t_go_straight + t_loop
            _p0 = p0 + Δp
            p_cmd = _p0 + ((v0_norm+ΔV)*(t-_t0) - 0.5*a*(t-_t0)^2) * Δp_unit
        else
            # stop
            _p0 = p0 + 2*Δp
            _t0 = t0 + t_go_straight + t_loop + t_go_straight
            p_cmd = _p0 + v0*(t-_t0)
        end
        return p_cmd
    end
end
