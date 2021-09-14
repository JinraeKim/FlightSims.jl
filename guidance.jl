using FlightSims
const FS = FlightSims

using LinearAlgebra  # for I, e.g., Matrix(I, n, n)
using ComponentArrays
using UnPack
using Transducers
using Plots
using DifferentialEquations  # for callbacks


struct GuidanceEnv <: AbstractEnv  # AbstractEnv exported from FS
    N
end

"""
FlightSims recommends you to use closures for State and Dynamics!. For more details, see https://docs.julialang.org/en/v1/devdocs/functions/.
"""
function State(env::GuidanceEnv)
    return function (p_M = zeros(3), v_M = zeros(3), p_T = zeros(3), v_T = zeros(3))
        ComponentArray(p_M = p_M, v_M = v_M, p_T = p_T, v_T = v_T)
    end
end

function Dynamics!(env::GuidanceEnv)
    # @unpack N = env  # @unpack is very useful!
    @Loggable function dynamics!(dx, x, params, t; u)  # `Loggable` makes it loggable via SimulationLogger.jl (imported in FS)
        @unpack p_M, v_M, p_T, v_T = x
        @log p_M  # to log p_M
        @log v_M  # to log v_M
        @log p_T

        dx.p_M = v_M
        dx.v_M = u
        dx.p_T = v_T
        dx.v_T = zeros(3)

        @onlylog r = norm(p_T-p_M)
        # @onlylog ṙ = dot(p_T-p_M, v_T-v_M) / r
    end
end

function PPNG(x, params, t)
    @unpack p_M, v_M, p_T, v_T = x
    N = 3

    Ω = cross(p_T-p_M, v_T-v_M) / dot(p_T-p_M, p_T-p_M)
    a_M = N * cross(Ω, v_M)
    return a_M
end


function main()
    # Initial condition
    p_M_0 = [0; 0; 0]
    V_0, γ_0, χ_0 = 300, 45/180*pi, 90/180*pi
    v_M_0 = V_0*[cos(γ_0)*sin(χ_0); cos(γ_0)cos(χ_0); sin(γ_0)]
    p_T_0 = [10E3; 5E3; 5E3]
    v_T_0 = 100*[cos(0)*sin(-pi/2); cos(0)cos(-pi/2); 0]

    # Design parameters
    N   = 3
    env = GuidanceEnv(N)
    # Simulation parameters
    Δt  = 0.001

    # callbacks
    function condition_stop(u, t, integrator)
        @unpack p_M, p_T, v_M, v_T = u
        r = norm(p_T-p_M)
        ṙ = dot(p_T-p_M, v_T-v_M) / r
        r - 0.1 #|| (r < 10 && ṙ >= 0)
    end
    affect!(integrator) = terminate!(integrator)  # See DiffEq.jl documentation
    cb_stop    = ContinuousCallback(condition_stop, affect!)
    cb = CallbackSet(cb_stop)  # useful for multiple callbacks
        
    # Execute Simulation
    # prob: DE problem, df: DataFrame
    x0 = State(env)(p_M_0, v_M_0, p_T_0, v_T_0)
    @time prob, df = sim(
                         x0,  # initial condition
                         apply_inputs(Dynamics!(env); u = PPNG);  # dynamics!; apply_inputs is exported from FS and is so useful for systems with inputs
                         tf=50.0,
                         savestep=Δt,  # savestep is NOT simulation step
                         solver=Tsit5(),
                         callback=cb,
                         reltol=1e-6
                        )  # sim is exported from FS
    ts = df.time
    p_Ms = df.sol |> Map(datum -> datum.p_M) |> collect
    p_Ts = df.sol |> Map(datum -> datum.p_T) |> collect

    p_Ms = hcat(p_Ms...)'
    p_Ts = hcat(p_Ts...)'


    # plot
    Fig_3D = plot(p_Ms[:,1], p_Ms[:,2], p_Ms[:,3], label="Missile")
    plot!(p_Ts[:,1], p_Ts[:,2], p_Ts[:,3], label="Target", lw=2,ls=:dash, legend=:bottomright)
    display(Fig_3D)

    # save
    dir_log = "figures"
    mkpath(dir_log)
    savefig(Fig_3D, joinpath(dir_log, "Fig_3D.pdf"))
    display(Fig_3D)
end

main()

