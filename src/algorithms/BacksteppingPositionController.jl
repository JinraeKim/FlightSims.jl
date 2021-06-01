"""
# References
- Controller (it may be modified in this implementation)
[1] G. P. Falconi and F. Holzapfel,
“Adaptive Fault Tolerant Control Allocation for a Hexacopter System,”
Proc. Am. Control Conf., vol. 2016-July, pp. 6760–6766, 2016.
- Reference model (e.g., xd, vd, ad, ad_dot, ad_ddot)
[2] S. J. Su, Y. Y. Zhu, H. R. Wang, and C. Yun, “A Method to Construct a Reference Model for Model Reference Adaptive Control,” Adv. Mech. Eng., vol. 11, no. 11, pp. 1–9, 2019.
"""
struct BacksteppingPositionController
    Ap
    Bp
    P
    Kp
    Kxd
    Kvd
    Kad
    Kȧd
    Käd
    Kt
    Kω
    function BacksteppingPositionController(m::Real)
        @assert m > 0
        Kxd = Diagonal(1.0*ones(3))
        Kvd = Diagonal(3.4*ones(3))
        Kad = Diagonal(5.4*ones(3))
        Kȧd = Diagonal(4.9*ones(3))
        Käd = Diagonal(2.7*ones(3))
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
        new(Ap, Bp, P, Kp, Kxd, Kvd, Kad, Kȧd, Käd, Kt, Kω)
    end
end

function State(env::BacksteppingPositionController)
    return function (pos0, m, g, vd=zeros(3), ad=zeros(3), ȧd=zeros(3), äd=zeros(3))
        @assert m > 0
        xd = pos0
        Td = m*g
        ComponentArray(xd=xd, vd=vd, ad=ad, ȧd=ȧd, äd=äd, Td=Td)
    end
end

function dynamics!(env::BacksteppingPositionController)
    @unpack Kxd, Kvd, Kad, Kȧd, Käd = env
    return function (dx_controller, x_controller, p_controller, t; pos_cmd, Ṫd)
        @unpack xd, vd, ad, ȧd, äd, Td = x_controller
        dx_controller.xd = vd
        dx_controller.vd = ad
        dx_controller.ad = ȧd
        dx_controller.ȧd = äd
        dx_controller.äd = (-Kxd*xd - Kvd*vd - Kad*ad - Kȧd*ȧd - Käd*äd + Kxd*pos_cmd)
        dx_controller.Td = Ṫd
        nothing
    end
end

function command(env::BacksteppingPositionController)
    T_u_inv(T) = [   0 1/T  0;
                  -1/T   0  0;
                     0   0 -1]
    T_u_inv_dot(T, Ṫ) = [    0 -Ṫ/T^2 0;
                         Ṫ/T^2      0 0;
                             0      0 0]
    T_ω(T) = [0 -T  0;
              T  0  0;
              0  0  0]
    skew(x) = [    0  -x[3]   x[2];
                x[3]      0  -x[1];
               -x[2]   x[1]     0]
    return function(p, v, R, ω,
                    xd, vd, ad, ȧd, äd, Td,
                    m::Real, J, g::Real,)
        @unpack Ap, Bp, P, Kp, Kt, Kω = env
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
        u2 = T_u_inv(T) * R * (2*Bp' * P * ep + u̇1 + Kt*et)
        Ṫd = u2[end]  # third element
        Ṫ = Ṫd  # TODO: no lag
        żB = -R' * T_ω(1.0) * ω
        ėt = u̇1 + u2[end]*zB + Td*żB
        ëp = Ap*ėp + Bp*ėt
        ü1 = m*äd + Kp*ëp
        Ṙ = -skew(ω) * R
        u̇2 = (
              (T_u_inv_dot(T, Ṫ)*R + T_u_inv(T)*Ṙ) * (2*Bp'*P*ep + u̇1 + Kt*et)
              + T_u_inv(T) * R * (2*Bp'*P*ėp + ü1 + Kt*ėt)
             )
        ω̇d = [1 0 0;
              0 1 0;
              0 0 0] * u̇2
        ωd = [u2[1:2]..., 0]
        eω = ωd - ω
        Md = cross(ω, J*ω) + J*(T_ω(T)'*R*et + ω̇d + Kω*eω)
        # νd = vcat(Td, Md)
        νd = ComponentArray(f=Td, M=Md)
        νd, Ṫd
    end
end