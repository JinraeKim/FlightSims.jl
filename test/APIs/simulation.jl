using FlightSims
using LinearAlgebra
using ControlSystems: lqr
using DifferentialEquations
using DataFrames
using Transducers


function test()
    n = 3
    Dynamics!(dx, x, p, t) = dx .= -p*x
    x0 = ones(n)
    p0 = 1.0
    tf = -1.00
    Δt = -0.01
    Δt2 = -0.004
    affect!(integrator) = integrator.p = 0.99*integrator.p
    affect2!(integrator) = integrator.p = 0.99*integrator.p
    cb_update = PeriodicCallback(affect!, Δt; initial_affect=false)
    cb_update2 = PeriodicCallback(affect2!, Δt2; initial_affect=false)
    function datum_format(x, t, integrator)
        p = copy(integrator.p)
        x = copy(x)
        t > 0.5 ? (; p=p) : (; x=x, p=p)  # varying keys
    end
    cb = CallbackSet(cb_update, cb_update2)
    prob, sol, df = sim(x0, Dynamics!, p0;
                        tf=tf, callback=cb,
                        datum_format=datum_format)
    df
end
