using Test


@testset "run-FlightSims-simulations" begin
    include("lqr.jl")
    include("environments/integrated_environments/backstepping_position_controller_static_allocator_multicopter.jl")
end
