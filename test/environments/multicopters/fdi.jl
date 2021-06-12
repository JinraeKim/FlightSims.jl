using FlightSims
const FS = FlightSims
using LinearAlgebra
using Transducers
using Plots


function test()
    τ = 0.2
    fdi = FS.LPFFDI(τ)
    m = 6
    Λ̂0 = State(fdi)(m)  # input dimension -> Diagonal(ones(m))
    Λ = function (x, p, t)
        Λ = Diagonal(ones(m))
        if t < 5
            Λ = Diagonal(0.5*ones(m))
        elseif t < 10
            Λ = Diagonal(0.1*ones(m))
        else
            Λ = Diagonal(0.0*ones(m))
        end
        Λ
    end
    prob, sol = sim(Λ̂0, apply_inputs(Dynamics!(fdi); Λ=Λ); tf=10.0)
    df = Process()(prob, sol)
    Λ̂s_diag = df.state |> Map(diag) |> collect
    plot(df.time, hcat(Λ̂s_diag...)')
end
