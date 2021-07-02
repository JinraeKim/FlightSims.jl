"""
Proportional-integral-derivative (PID) controller.
# Notes
The derivative term is obtained via second-order filter.
For more details, see [1] and singular perturbation theory.
# References
[1] J. A. Farrell, M. Polycarpou, M. Sharma, and W. Dong, “Command Filtered Backstepping,” IEEE Transactions on Automatic Control, vol. 54, no. 6, pp. 1391–1395, Jun. 2009, doi: 10.1109/TAC.2009.2015562.
"""
struct PID <: AbstractEnv
    K_P
    K_I
    K_D
    ω_n
    ζ
    windup_limit
    function PID(K_P, K_I, K_D; ω_n=1e1, ζ=5e-1, windup_limit=Inf)
        @assert windup_limit > 0.0
        @assert ω_n > 0.0
        @assert ζ > 0.0
        new(K_P, K_I, K_D, ω_n, ζ, windup_limit)
    end
end

"""
∫e: integral of the error
ê: the estimate of the error
ė̂: the estimate of the error rate
"""
function State(controller::PID)
    function state(; ∫e=zeros(1), ê=zeros(1), ė̂=zeros(1))
        ComponentArray(∫e=∫e, ê=ê, ė̂=ė̂)
    end
end

function Dynamics!(controller::PID)
    @unpack ω_n, ζ, windup_limit = controller
    @Loggable function dynamics!(dx, x, p, t; e)
        @assert !(typeof(e) <: Number)  # make sure that `e` is a 1-d array
        @unpack ∫e, ê, ė̂ = x
        @onlylog e, ∫e, ê, ė̂
        if norm(∫e) < windup_limit
            dx.∫e .= e
        elseif norm(∫e) > windup_limit && all(sign.(e) .* sign.(∫e) .< zero.(e))
            dx.∫e .= e
        else
            dx.∫e .= zero.(e)
        end
        dx.ê .= ω_n*ė̂
        dx.ė̂ = -2*ζ*ω_n*ė̂ - ω_n*(ê - e)
    end
end

function Command(controller::PID)
    @unpack K_P, K_I, K_D = controller
    function command(e, ∫e, ė̂)
        -(K_P*e + K_I*∫e + K_D*ė̂)
    end
end
