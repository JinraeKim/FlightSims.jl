abstract type AbstractLQR end

"""
Infinite-horizon continuous-time linear quadratic regulator (LQR).
"""
struct LQR <: AbstractLQR
    A
    B
    Q
    R
end

function RunningCost(lqr::LQR)
    @unpack Q, R = lqr
    return function (x, u)
        x'*Q*x + u'*R*u
    end
end

function ARE_solution(lqr::LQR)
    @unpack A, B, Q, R = lqr
    P, _, _ = MatrixEquations.arec(A, B*inv(R)*B', Q)
    P
end

function optimal_gain(lqr::LQR)
    @unpack B, R = lqr
    P = ARE_solution(lqr)
    K = inv(R) * B' * P
end

"""
Minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
"""
function OptimalController(lqr::LQR)
    K = optimal_gain(lqr)
    return function (x, p, t)
        -K*x
    end
end

function solutions(lqr::LQR)
    P = ARE_solution(lqr)
    K = optimal_gain(lqr)
    optimal_controller = OptimalController(lqr)
    P, K, optimal_controller
end
