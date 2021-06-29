"""
Proportional-integral-derivative (PID) controller.
Note that the derivative is obtained via low-pass-filter (LPF).
For more details, see singular perturbation theory.
"""
struct PID <: AbstractEnv
    K_P
    K_I
    K_D
    τ
    function PID(K_P, K_I, K_D; τ=1e-2)
        new(K_P, K_I, K_D, τ)
    end
end

"""
ê: the estimate of the error
∫e: integral of the error
"""
function State(controller::PID)
    function state(; ∫e=zeros(1), ê=zeros(1))
        ComponentArray(∫e=∫e, ê=ê)
    end
end

"""
Note: ê is estimated via LPF
"""
function Dynamics!(controller::PID)
    @unpack τ = controller
    @Loggable function dynamics!(dx, x, p, t; e)
        @assert typeof(e) != Number
        @unpack ∫e, ê = x
        @onlylog e, ∫e, ê
        dx.∫e .= e
        dx.ê .= deriv_ê(controller, e, ê)
    end
end

function deriv_ê(controller::PID, e, ê)
    @unpack τ = controller
    (e - ê) / τ
end

function Command(controller::PID)
    @unpack K_P, K_I, K_D = controller
    function command(e, ∫e, ê)
        -(K_P*e + K_I*∫e + K_D*deriv_ê(controller, e, ê))
    end
end
