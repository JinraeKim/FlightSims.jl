"""
Moore-Penrose inverse control allocator.
"""
struct PseudoInverseControlAllocator <: AbstractControlAllocator
    B_inv
    function PseudoInverseControlAllocator(B)
        B_inv = pinv(B)
        new(B_inv)
    end
end

"""
# Variables
ν: virtual input
# Notes
ν = B*u where u: control input
"""
(mx::PseudoInverseControlAllocator)(ν) = mx.B_inv * ν
