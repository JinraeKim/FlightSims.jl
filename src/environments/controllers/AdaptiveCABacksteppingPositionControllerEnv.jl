"""
# References
- Controller (modified for non-FTC (fault tolerant control))
[1] G. P. Falconi and F. Holzapfel,
“Adaptive Fault Tolerant Control Allocation for a Hexacopter System,”
Proc. Am. Control Conf., vol. 2016-July, pp. 6760–6766, 2016.
"""
struct AdaptiveCABacksteppingPositionControllerEnv <: AbstractEnv
    baseline::BacksteppingPositionControllerEnv
    γ
    function AdaptiveCABacksteppingPositionControllerEnv(args...; γ=1e-1, kwargs...)
        baseline = BacksteppingPositionControllerEnv(args...; kwargs...)
        new(baseline, γ)
    end
end

"""
(n_input::Int=6) default settings for hexacopters
"""
function State(controller::AdaptiveCABacksteppingPositionControllerEnv)
    @unpack baseline = controller
    return function (args...; n_input::Int=6)
        x_baseline = State(baseline)(args...)
        Θ̂=zeros(n_input, 4)  # output: virtual input consisting of [f, M...] where `f`: total thrust, `M`: moment
        ComponentArray(baseline=x_baseline, Θ̂=Θ̂)
    end
end

function dynamics!(controller::AdaptiveCABacksteppingPositionControllerEnv)
    @unpack baseline = controller
    return function (dX, X, p, t; pos_cmd=nothing, Ṫd, Θ̂̇)
        dynamics!(baseline)(dX.baseline, X.baseline, (), t; pos_cmd=pos_cmd, Ṫd=Ṫd)
        dX.Θ̂ = Θ̂̇
        nothing
    end
end

function Proj(controller::AdaptiveCABacksteppingPositionControllerEnv, θ, y; ϵ=1e-2, θ_max=1e5)
    f = ((1+ϵ) * norm(θ)^2 - θ_max^2) / (ϵ * θ_max^2)
    ∇f = (2*(1+ϵ) / (ϵ * θ_max^2)) * θ
    proj = nothing
    if f > 0 && (∇f)' * y > 0
        ∇f_unit = ∇f / norm(∇f)
        proj = y - dot(∇f_unit, y) * f * ∇f_unit
    else
        proj = y
    end
    return proj
end
function Proj_R(controller::AdaptiveCABacksteppingPositionControllerEnv, C, Y)
    proj_R = zeros(size(Y))
    for i in 1:size(Y)[1]
        ci = C[i, :]
        yi = Y[i, :]
        proj_R[i, :] = Proj(controller, ci, yi)
    end
    return proj_R
end

function command(controller::AdaptiveCABacksteppingPositionControllerEnv)
    @unpack baseline, γ = controller
    @unpack Ap, Bp, P, Kp, Kt = baseline
    return function (p, v, R, ω,
                     xd, vd, ad, ȧd, äd, Td, Θ̂,
                     m::Real, J, g::Real, B_A,)
        _cmd = _command(baseline)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g)
        @unpack νd, Ṫd, e, zB, T = _cmd
        P̄ = [          P          zeros(6, 6);
             zeros(6, 6)  0.5*Matrix(I, 6, 6)]
        θ_ėp = Bp
        θ_u̇1 = Kp * θ_ėp
        θ_ėt = θ_u̇1
        θ_ëp = Ap * θ_ėp + Bp * θ_ėt
        θ_ü1 = Kp * θ_ëp
        θ_u̇2 = T_u_inv(baseline, T) * R * (2*Bp' * P * θ_ėp
                                           + θ_ü1 + Kt*θ_ėt)
        θ_ω̇d = [1 0 0;
                0 1 0;
                0 0 0] * θ_u̇2
        B̄ = [-θ_ėp*zB  zeros(6, 3);
             -θ_u̇1*zB  zeros(3, 3);
             -θ_ω̇d*zB  inv(J)]
        Θ̂̇ = γ * Proj_R(controller, Θ̂, (νd * e' * P̄ * B̄ * B_A)')
        return νd, Ṫd, Θ̂̇
    end
end
