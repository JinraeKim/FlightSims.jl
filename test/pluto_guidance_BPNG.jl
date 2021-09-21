### A Pluto.jl notebook ###
# v0.16.1

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
# 2D Guidance Simulation
Biased Proportional Navigation Guidance Law is implemented.

Command is given as follows:

$\displaystyle a_{cmd} = NV\dot{\lambda} - \frac{K_{r}\left(r\right)K_{\eta}\left(\eta\right)}{r}e_{\gamma_{f}}$
"""

# ╔═╡ 50becda1-a0e6-4e4c-98ca-a72992a5fc42
begin
function main(γ_0::Number, bpng::BPNG)
    # Initial condition
    p_M_0 = [0; 0]
    V_M_0 = 300
    v_M_0 = V_M_0*[cos(γ_0); sin(γ_0)]
    p_T_0 = [5E3; 0]
    v_T_0 = [0; 0]

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

	return df
end
end

# ╔═╡ c5b50f97-f3b7-42a6-8064-84a10de2a099
md"""
slider 1: $\alpha$
"""

# ╔═╡ 9fcf0c9b-d559-402f-883c-c53f61815c94
@bind α Slider(0.01:0.01:1; default=1, show_value=true)

# ╔═╡ 6390ff9a-cfa3-4905-94bf-52647f3dd0b1
md"""
slider 2: $n$
"""

# ╔═╡ d4b9aafb-a583-4a83-937b-91965cf607ab
@bind n Slider(0 : 0.1 : 10; default=1, show_value=true)

# ╔═╡ abd776cd-90b5-4074-ad53-899522bf02c5
md"""
slider 3: $\gamma_{0}$
"""

# ╔═╡ 5036f737-2b04-434c-ac2a-232c57eb943b
@bind γ_0_deg Slider(0:1:90; default=30, show_value=true)

# ╔═╡ ae874a64-9a88-446a-9c2e-34b8125655aa
md"""
slider 4: $\gamma_{f_{d}}$
"""

# ╔═╡ 6afff3ea-6d91-42e9-b657-0b956bef56a0
@bind γ_f_d_deg Slider(-90:1:90; default=-90, show_value=true)

# ╔═╡ 4ebf257d-c3d8-466b-9667-9f0728c203ba
md"""
slider 5: $\sigma_{\lim}$
"""

# ╔═╡ 8f18d2e1-1972-4f7a-8d73-40bb9b744227
@bind σ_M_lim_deg Slider(0:1:90; default=60, show_value=true)

# ╔═╡ bce435b0-c6d4-4ac7-93aa-727ed08c73e5
begin
	# Design parameters
    N       = 3
    δ       = 0.01
    A_M_max = 100
	bpng = BPNG(N, α, δ, deg2rad(γ_f_d_deg), deg2rad(σ_M_lim_deg), A_M_max, n)
	
	# Execute simulation
	df = main(deg2rad(γ_0_deg), bpng)
	
	ts = df.time
    p_Ms = df.sol |> Map(datum -> datum.pursuer.p) |> collect
    p_Ts = df.sol |> Map(datum -> datum.evador.p)  |> collect

    p_Ms = hcat(p_Ms...)'
    p_Ts = hcat(p_Ts...)'
	
	# Plotting
	f = fig_print(p_Ms[:,1], p_Ms[:,2], "Traj_2D", "Missile", "y [m]"; xlabel_string = "x [m]", ar_val = :equal, save_file = 0)
	scatter!(f, p_Ts[:,1], p_Ts[:,2], label = "Target")
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
# ╟─ae874a64-9a88-446a-9c2e-34b8125655aa
# ╟─6afff3ea-6d91-42e9-b657-0b956bef56a0
# ╟─4ebf257d-c3d8-466b-9667-9f0728c203ba
# ╟─8f18d2e1-1972-4f7a-8d73-40bb9b744227
# ╠═bce435b0-c6d4-4ac7-93aa-727ed08c73e5
