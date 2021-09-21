using FlightSims
const FS = FlightSims
using LinearAlgebra
using ComponentArrays
using UnPack
using Transducers
using Plots
using DifferentialEquations  # for callbacks


function main(γ_0::Number, γ_f_d::Number)
    # Initial condition
    p_M_0 = [0; 0]
    V_M_0 = 300
    v_M_0 = V_M_0*[cos(γ_0); sin(γ_0)]
    p_T_0 = [5E3; 0]
    v_T_0 = [0; 0]

    # Design parameters
    N       = 3
    α       = 1
    δ       = 0.01
    σ_M_lim = deg2rad(60)
    A_M_max = 100

    # Simulation parameters
    Δt  = 0.01

    # callbacks
    function condition_stop(u, t, integrator)
        p_M = u.pursuer.p
        v_M = u.pursuer.v
        p_T = u.evador.p
        v_T = u.evador.v
        r = norm(p_T-p_M)
        ṙ = dot(p_T-p_M, v_T-v_M) / r
        r < 0.1  #|| (r < 10 && ṙ >= 0)
    end
    affect!(integrator) = terminate!(integrator)  # See DiffEq.jl documentation
    cb_stop    = DiscreteCallback(condition_stop, affect!)
    cb = CallbackSet(cb_stop)  # useful for multiple callbacks
        
    # Execute Simulation
    pursuer = PointMass2DMissile()
    evador = PointMass2DMissile()
    env = PursuerEvador2DMissile(pursuer, evador)
    # prob: DE problem, df: DataFrame
    x0_pursuer = State(pursuer)(p_M_0, v_M_0)
    x0_evador = State(evador)(p_T_0, v_T_0)
    x0 = State(env)(x0_pursuer, x0_evador)
    bpng = BPNG(N, α, δ, γ_f_d, σ_M_lim, A_M_max)
    function GuidanceLaw(bpng::BPNG)
        bpng_law = Command(bpng)
        return function (x, params, t)
            p_M = x.pursuer.p
            v_M = x.pursuer.v
            p_T = x.evador.p
            v_T = x.evador.v
            bpng_law(p_M, v_M, p_T, v_T)
        end
    end
    @time prob, df = sim(
                         x0,  # initial condition
                         apply_inputs(Dynamics!(env);
                                      u_pursuer=GuidanceLaw(bpng),
                                      u_evador=(x, params, t) -> zeros(2));  # dynamics!; apply_inputs is exported from FS and is so useful for systems with inputs
                         tf=50.0,
                         savestep=Δt,  # savestep is NOT simulation step
                         solver=Tsit5(),
                         callback=cb,
                         reltol=1e-6
                        )  # sim is exported from FS
    ts = df.time
    p_Ms = df.sol |> Map(datum -> datum.pursuer.p) |> collect
    p_Ts = df.sol |> Map(datum -> datum.evador.p) |> collect

    p_Ms = hcat(p_Ms...)'
    p_Ts = hcat(p_Ts...)'

    # return p_Ms, p_Ts

    # plot
    Fig_2D = plot(p_Ms[:,1], p_Ms[:,2], label="Missile")
    plot!(p_Ts[:,1], p_Ts[:,2], label="Target", lw=2, markershape=:circle, legend=:bottomright)

    # save
    dir_log = "figures"
    mkpath(dir_log)
    savefig(Fig_2D, joinpath(dir_log, "Fig_2D.pdf"))
    display(Fig_2D)
end
