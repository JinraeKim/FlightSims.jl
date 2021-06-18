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
    tf = 10.0
    # @Loggable will generate a hidden dictionary (NEVER USE THE PRIVILEGED NAME, `__LOGGER_DICT__`)
    # @Loggable will also automatically return the privileged dictionary
    # @Loggable will also copy the state `x` to avoid view issue; https://diffeq.sciml.ai/stable/features/callback_library/#Constructor-5
    # @log will automatically log annotated data in the privileged dictionary
    @Loggable function dynamics!(dx, x, p, t; u)
        @log state = x
        @log input = u
        Dynamics!(env)(dx, x, p, t; u)  # predefined dynamics exported from FlightSims
        # NEVER RETURN SOMETHING; just mutate dx
    end
    prob, sol, df = sim(
                        x0,  # initial condition
                        apply_inputs(dynamics!; u=u_lqr);  # dynamics with input of LQR
                        tf=tf,  # final time
                        savestep=0.01,
                       )
    plot(df.time, hcat(df.state...)'; title="state variable", label=["x1" "x2"])  # Plots
    savefig("figures/x_lqr.png")
    plot(df.time, hcat(df.input...)'; title="control input", label="u")  # Plots
    savefig("figures/u_lqr.png")
end
