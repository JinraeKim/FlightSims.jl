using FlightSims
const FS = FlightSims
using LinearAlgebra
using Plots


function test()
    # linear system
    A = [0 1;
         0 0]
    B = [0;
         1]
    n, m = 2, 1
    env = LinearSystemEnv(A, B)  # exported from FlightSims
    x0 = State(env)([1.0, 2.0])
    # optimal control
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    lqr = LQR(A, B, Q, R)  # exported from FlightSims
    u_lqr = FS.OptimalController(lqr)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
    # simulation
    # case 1: processing data simultaneously
    tf = 10.0
    # prob, sol = sim(
    @LOG function dynamics!(dx, x, p, t)
        u = u_lqr(x, p, t)
        _u = length(u) == 1 ? u[1] : u
        @log time = t
        @log state = x
        @log input = _u
        dx .= A*x + B*_u
        nothing
    end
    prob, sol, df = sim(
                        x0,  # initial condition
                        # apply_inputs(Dynamics!(env); u=u_lqr);  # dynamics with input of LQR
                        # apply_inputs(dynamics!; u=u_lqr);  # dynamics with input of LQR
                        dynamics!;  # dynamics with input of LQR
                        tf=tf,  # final time
                        log_func=dynamics!__LOG__,
                        # datum_format=save_inputs(DatumFormat(env); input=u_lqr),  # saving data; default key: time, state
                        # log_off=true,
                        savestep=0.01,
                       )
    # case 2: processing data after simulation
    # df = Process(env)(prob, sol; savestep=0.01)  # DataFrame; `savestep` means data sampling period.
    plot(df.time, hcat(df.state...)'; title="state variable", label=["x1" "x2"])  # Plots
    savefig("figures/x_lqr.png")
    plot(df.time, hcat(df.input...)'; title="control input", label="u")  # Plots
    savefig("figures/u_lqr.png")
end
