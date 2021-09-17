### A Pluto.jl notebook ###
# v0.16.0

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : missing
        el
    end
end

# ╔═╡ 02329bbc-15a4-11ec-2522-df175bfb53e0
begin
    import Pkg
    # careful: this is _not_ a reproducible environment
    # activate the global environment
    Pkg.activate()

    using Plots, PlutoUI, LinearAlgebra
	using FlightSims
	const FS = FlightSims
	using ComponentArrays
	using UnPack
	using Transducers
	using DifferentialEquations  # for callbacks
end

# ╔═╡ b4aa1cc3-07de-4717-a39e-8a9c4b1c8512
md"""
# 3D Guidance Simulation
Pure Proportional Navigation Guidance Law is implemented.
"""

# ╔═╡ 50becda1-a0e6-4e4c-98ca-a72992a5fc42
begin
function main(N, V_0, χ_0)
    # Initial condition
    p_M_0 = [0; 0; 0]
    γ_0 = pi/4
    v_M_0 = V_0*[cos(γ_0)*sin(χ_0); cos(γ_0)cos(χ_0); sin(γ_0)]
    p_T_0 = [10E3; 5E3; 5E3]
    v_T_0 = 100*[cos(0)*sin(-pi/2); cos(0)cos(-pi/2); 0]

    # Design parameters
    
    # Simulation parameters
    Δt  = 0.01

    # callbacks
    function condition_stop(u, t, integrator)
        # @unpack p_M, p_T, v_M, v_T = u
        p_M = u.pursuer.p
        v_M = u.pursuer.v
        p_T = u.evador.p
        v_T = u.evador.v
        r = norm(p_T-p_M)
        ṙ = dot(p_T-p_M, v_T-v_M) / r
        r < 1  || (r < 10 && ṙ >= 0)
    end
    affect!(integrator) = terminate!(integrator)  # See DiffEq.jl documentation
    cb_stop    = DiscreteCallback(condition_stop, affect!)
    cb = CallbackSet(cb_stop)  # useful for multiple callbacks
        
    # Execute Simulation
    pursuer = PointMass3DMissile()
    evador = PointMass3DMissile()
    env = PursuerEvador3DMissile(pursuer, evador)
    # prob: DE problem, df: DataFrame
    x0_pursuer = State(pursuer)(p_M_0, v_M_0)
    x0_evador = State(evador)(p_T_0, v_T_0)
    x0 = State(env)(x0_pursuer, x0_evador)
    ppng = PPNG(N)
    function GuidanceLaw(ppng::PPNG)
        ppng_law = Command(ppng)
        return function (x, params, t)
            p_M = x.pursuer.p
            v_M = x.pursuer.v
            p_T = x.evador.p
            v_T = x.evador.v
            ppng_law(p_M, v_M, p_T, v_T)
        end
    end
    @time prob, df = sim(
                         x0,  # initial condition
                         apply_inputs(Dynamics!(env);
                                      u_pursuer=GuidanceLaw(ppng),
                                      u_evador=(x, params, t) -> zeros(3));  # dynamics!; apply_inputs is exported from FS and is so useful for systems with inputs
                         tf=100.0,
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

		
	return p_Ms, p_Ts
end
end

# ╔═╡ c5b50f97-f3b7-42a6-8064-84a10de2a099
md"""
slider 1: $N$
"""

# ╔═╡ 9fcf0c9b-d559-402f-883c-c53f61815c94
@bind N Slider(1:0.05:5; default=3, show_value=true)

# ╔═╡ 6390ff9a-cfa3-4905-94bf-52647f3dd0b1
md"""
slider 2: $\chi_0$
"""

# ╔═╡ d4b9aafb-a583-4a83-937b-91965cf607ab
@bind χ_0_deg Slider(-90 : 0.1 : 90; default=90, show_value=true)

# ╔═╡ abd776cd-90b5-4074-ad53-899522bf02c5
md"""
slider 3: $V_0$
"""

# ╔═╡ 5036f737-2b04-434c-ac2a-232c57eb943b
@bind V_0 Slider(100:1:500; default=300, show_value=true)

# ╔═╡ bce435b0-c6d4-4ac7-93aa-727ed08c73e5
begin
	p_Ms, p_Ts = main(N, V_0, deg2rad(χ_0_deg))
	
	# plot
	Fig_3D = plot(p_Ms[:,1], p_Ms[:,2], p_Ms[:,3], label="Missile", lw=2)
	plot!(Fig_3D, p_Ts[:,1], p_Ts[:,2], p_Ts[:,3], label="Target", lw=2,ls=:dot, legend=:bottomright)
	plot!(Fig_3D, xlabel = "x", ylabel = "y", zlabel = "z", foreground_color_grid=:grey, camera = (20, 70))
	
end

# ╔═╡ 8d8c92d8-2b04-4568-9ac2-97f4074baf0a
begin
	dir_log = "figures"
    mkpath(dir_log)
    savefig(Fig_3D, joinpath(dir_log, "Fig_3D.png"))
    display(Fig_3D)
end

# ╔═╡ Cell order:
# ╟─b4aa1cc3-07de-4717-a39e-8a9c4b1c8512
# ╟─02329bbc-15a4-11ec-2522-df175bfb53e0
# ╟─50becda1-a0e6-4e4c-98ca-a72992a5fc42
# ╟─c5b50f97-f3b7-42a6-8064-84a10de2a099
# ╟─9fcf0c9b-d559-402f-883c-c53f61815c94
# ╟─6390ff9a-cfa3-4905-94bf-52647f3dd0b1
# ╟─d4b9aafb-a583-4a83-937b-91965cf607ab
# ╟─abd776cd-90b5-4074-ad53-899522bf02c5
# ╟─5036f737-2b04-434c-ac2a-232c57eb943b
# ╠═bce435b0-c6d4-4ac7-93aa-727ed08c73e5
# ╟─8d8c92d8-2b04-4568-9ac2-97f4074baf0a
