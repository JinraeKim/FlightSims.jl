abstract type AbstractMixer end

"""
Moore-Penrose inverse (pseudo inverse) mixer.
"""
struct Mixer <: AbstractMixer
    B_inv
    function Mixer(B)
        B_inv = pinv(B)
        new(B_inv)
    end
end
"""
# Variables
f ∈ R: total thurst
M ∈ R^3: moment
ν: [f, M...]
# Notes
ν = B*u
"""
(mx::Mixer)(ν) = mx.B_inv * ν
