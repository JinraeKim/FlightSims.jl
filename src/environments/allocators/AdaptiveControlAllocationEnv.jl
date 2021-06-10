"""
# References
- Controller
[1] G. P. Falconi and F. Holzapfel,
“Adaptive Fault Tolerant Control Allocation for a Hexacopter System,”
Proc. Am. Control Conf., vol. 2016-July, pp. 6760–6766, 2016.
- Reference model (e.g., xd, vd, ad, ad_dot, ad_ddot)
[2] S. J. Su, Y. Y. Zhu, H. R. Wang, and C. Yun, “A Method to Construct a Reference Model for Model Reference Adaptive Control,” Adv. Mech. Eng., vol. 11, no. 11, pp. 1–9, 2019.
"""
struct AdaptiveControlAllocationEnv <: AbstractEnv
    γ
    function AdaptiveControlAllocationEnv(baseline; γ=1e-1)
        new(baseline, γ)
    end
end

function State(allocator::AdaptiveControlAllocationEnv)
    return function (; Θ̂=zeros(6, 4))
        ComponentArray(Θ̂=Θ̂)
    end
end

function Dynamics!(allocator::AdaptiveControlAllocationEnv)
    return function (dx, x, p, t; Θ̂̇)
        dx.Θ̂ .= Θ̂̇
    end
end
