"""
# Variables
- ϕ: roll
- θ: pitch
- ψ: yaw
# Notes
- RotXYZ (Z first applied)
- `R` denotes the rotation matrix from I (inertial) to B (body).
"""
function euler(R::Rotations.RotXYZ)
    ϕ = R.theta1  # X
    θ = R.theta2  # Y
    ψ = R.theta3  # Z
    [ϕ, θ, ψ]
end
