using FlightSims
const FS = FlightSims
using LinearAlgebra
using Plots


function test()
    # linear system
    n, m = 2, 1
    env = LinearSystemEnv(n, m)  # exported from FlightSims
    x0 = State(env)([1.0, 2.0])
    A = [0 1;
         0 0]
    B = [0;
         1]
    p = Params(env)(A, B)
    # optimal control
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    lqr = LQR(A, B, Q, R)  # exported from FlightSims
    u_lqr = FS.OptimalController(lqr)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
    # simulation
    t0, tf = 0.0, 10.0
    Δt = 0.01  # save period; not simulation time step
    # case 1: processing data simultaneously
    prob, sol, df = sim(
                        x0,  # initial condition
                        apply_inputs(Dynamics!(env); u=u_lqr),  # dynamics with input of LQR
                        p;
                        t0=t0, tf=tf,  # final time
                        datum_format=save_inputs(DatumFormat(env); input=u_lqr),  # saving data; default key: time, state
                        saveat=t0:Δt:tf,
                       )
    # case 2: processing data after simulation
    # df = Process(env)(prob, sol; Δt=0.01)  # DataFrame; `Δt` means data sampling period.
    plot(df.time, hcat(df.state...)'; title="state variable", label=["x1" "x2"])  # Plots
    savefig("figures/x_lqr.png")
    plot(df.time, hcat(df.input...)'; title="control input", label="u")  # Plots
    savefig("figures/u_lqr.png")
end
