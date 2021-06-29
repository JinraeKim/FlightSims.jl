"""
Moore-Penrose inverse control allocator.
"""
struct PseudoInverseAllocator <: StaticAllocator
    B_pinv
    function PseudoInverseAllocator(B)
        B_pinv = pinv(B)
        new(B_pinv)
    end
end

"""
# Variables
ν: virtual input
# Notes
ν = B*u where u: control input
"""
function (allocator::PseudoInverseAllocator)(ν, Λ=Diagonal(ones(size(ν))))
    (pinv(Λ) * allocator.B_pinv) * ν  # pinv(B*Λ) = pinv(Λ) * pinv(B)
end
