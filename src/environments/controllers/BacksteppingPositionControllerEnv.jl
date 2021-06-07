"""
# References
- Controller (modified for non-FTC (fault tolerant control))
[1] G. P. Falconi and F. Holzapfel,
“Adaptive Fault Tolerant Control Allocation for a Hexacopter System,”
Proc. Am. Control Conf., vol. 2016-July, pp. 6760–6766, 2016.
"""
struct BacksteppingPositionControllerEnv <: AbstractEnv
    Ref_model
    Ap
    Bp
    P
    Kp
    Kt
    Kω
    function BacksteppingPositionControllerEnv(m::Real; x_cmd_func=nothing)
        @assert m > 0
        Ref_model = ReferenceModelEnv(4; x_cmd_func=x_cmd_func)
        # position
        Kx = m*1*Matrix(I, 3, 3)
        Kv = m*1*1.82*Matrix(I, 3, 3)
        Kp = hcat(Kx, Kv)
        Q = Diagonal(1*ones(6))
        # thrust
        Kt = Diagonal(4*ones(3))
        # angular
        Kω = Diagonal(20*ones(3))
        # reference model
        Ap = [zeros(3, 3) Matrix(I, 3, 3);
              -(1/m)*Kx -(1/m)*Kv]
        Bp = [zeros(3, 3);
              (1/m)*Matrix(I, 3, 3)]
        P = lyapc(Ap, Q)
        new(Ref_model, Ap, Bp, P, Kp, Kt, Kω)
    end
end

"""
# Notes
ref_model.x_0 = xd
ref_model.x_1 = vd
ref_model.x_2 = ad
ref_model.x_3 = ȧd
ref_model.x_4 = äd
"""
function State(controller::BacksteppingPositionControllerEnv)
    @unpack Ref_model = controller
    return function (pos0, m, g)
        @assert m > 0
        ref_model = State(Ref_model)(pos0)
        Td = m*g  # default
        ComponentArray(ref_model=ref_model, Td=Td)
    end
end

function Params(controller::BacksteppingPositionControllerEnv)
    return function ()
        @warn "TODO"
        nothing
    end
end

function dynamics!(controller::BacksteppingPositionControllerEnv)
    @unpack Ref_model = controller
    return function (dX, X, p, t; pos_cmd=nothing, Ṫd)
        dynamics!(Ref_model)(dX.ref_model, X.ref_model, (), t; x_cmd=pos_cmd)  # be careful; parameter = ()
        dX.Td = Ṫd
        nothing
    end
end

# utils; will also be used in AdaptiveCABacksteppingPositionControllerEnv
T_u_inv(controller::BacksteppingPositionControllerEnv, T) = [   0 1/T  0;
                                                             -1/T   0  0;
                                                                0   0 -1]
T_u_inv_dot(controller::BacksteppingPositionControllerEnv, T, Ṫ) = [    0 -Ṫ/T^2 0;
                                                                    Ṫ/T^2      0 0;
                                                                        0      0 0]
T_ω(controller::BacksteppingPositionControllerEnv, T) = [0 -T  0;
                                                         T  0  0;
                                                         0  0  0]
function _command(controller::BacksteppingPositionControllerEnv)
    @unpack Ap, Bp, P, Kp, Kt, Kω = controller
    return function(p, v, R, ω,
                    xd, vd, ad, ȧd, äd, Td,
                    m::Real, J, g::Real,)
        ex = xd - p
        ev = vd - v
        ep = vcat(ex, ev)
        # u1
        g_vec = [0, 0, g]
        u1 = m*(ad - g_vec) + Kp*ep
        zB = R' * [0, 0, 1]
        td = -Td * zB
        et = u1 - td
        ėp = Ap*ep + Bp*et
        u̇1 = m*ȧd + Kp*ėp
        T = Td  # TODO: no lag
        # u2
        u2 = T_u_inv(controller, T) * R * (2*Bp' * P * ep + u̇1 + Kt*et)
        Ṫd = u2[end]  # third element
        Ṫ = Ṫd  # TODO: no lag
        żB = -R' * T_ω(controller, 1.0) * ω
        ėt = u̇1 + u2[end]*zB + Td*żB
        ëp = Ap*ėp + Bp*ėt
        ü1 = m*äd + Kp*ëp
        Ṙ = -skew(copy(ω)) * R  # see `utils/skew.jl`
        u̇2 = (
              (T_u_inv_dot(controller, T, Ṫ)*R + T_u_inv(controller, T)*Ṙ) * (2*Bp'*P*ep + u̇1 + Kt*et)
              + T_u_inv(controller, T) * R * (2*Bp'*P*ėp + ü1 + Kt*ėt)
             )
        ω̇d = [1 0 0;
              0 1 0;
              0 0 0] * u̇2
        ωd = [u2[1:2]..., 0]
        eω = ωd - ω
        Md = cross(ω, J*ω) + J*(T_ω(controller, T)'*R*et + ω̇d + Kω*eω)
        # νd = vcat(Td, Md)
        νd = ComponentArray(f=Td, M=Md)
        ComponentArray(νd=νd, Ṫd=Ṫd, e=e, zB=zB)
    end
end

function command(controller::BacksteppingPositionControllerEnv)
    return function(p, v, R, ω,
                    xd, vd, ad, ȧd, äd, Td,
                    m::Real, J, g::Real,)
        _cmd = _command(controller)(p, v, R, ω, xd, vd, ad, ȧd, äd, Td, m, J, g,)
        _cmd.νd, _cmd.Ṫd
    end
end
