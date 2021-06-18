using FlightSims
const FS = FlightSims
using LinearAlgebra
using ComponentArrays
using Plots


function test()
    n, m = 2, 1
    # linear systems
    A1 = [0 1;
         0 0]
    B1 = [0;
         1]
    env1 = LinearSystemEnv(A1, B1)  # exported from FlightSims
    x01 = State(env1)([1.0, 2.0])
    A2 = [0 2;
         0 0]
    B2 = [0;
         2]
    env2 = LinearSystemEnv(A2, B2)  # exported from FlightSims
    x02 = State(env2)([3.0, -1.0])
    x0 = ComponentArray(sys1=x01, sys2=x02)
    # optimal control
    Q = Matrix(I, n, n)
    R = Matrix(I, m, m)
    lqr1 = LQR(A1, B1, Q, R)  # exported from FlightSims
    u_lqr1 = FS.OptimalController(lqr1)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
    lqr2 = LQR(A2, B2, Q, R)  # exported from FlightSims
    u_lqr2 = FS.OptimalController(lqr2)  # (x, p, t) -> -K*x; minimise J = ∫ (x' Q x + u' R u) from 0 to ∞
    # simulation
    tf = 10.0
    # @Loggable will generate a hidden dictionary (NEVER USE THE PRIVILEGED NAME, `__LOGGER_DICT__`)
    # @Loggable will also automatically return the privileged dictionary
    # @Loggable will also copy the state `x` to avoid view issue; https://diffeq.sciml.ai/stable/features/callback_library/#Constructor-5
    # @log will automatically log annotated data in the privileged dictionary
    @Loggable function dynamics1!(dx, x, p, t; u)
        @log state = x
        @log input = u
        Dynamics!(env1)(dx, x, (), t; u)
        # NEVER RETURN SOMETHING; just mutate dx
    end
    @Loggable function dynamics2!(dx, x, p, t; u)
        @log state = x
        @log input = u
        Dynamics!(env2)(dx, x, (), t; u)
        # NEVER RETURN SOMETHING; just mutate dx
    end
    @Loggable function dynamics!(dx, x, p, t)
        u1 = u_lqr1(x.sys1, (), t)
        u2 = u_lqr2(x.sys2, (), t)
        @nested_log :sys1 dynamics1!(dx.sys1, x.sys1, (), t; u=u1)  # predefined dynamics exported from FlightSims
        @nested_log :sys2 dynamics2!(dx.sys2, x.sys2, (), t; u=u2)  # predefined dynamics exported from FlightSims
        # NEVER RETURN SOMETHING; just mutate dx
    end
    prob, sol, df = sim(
                        x0,  # initial condition
                        dynamics!;  # dynamics with input of LQR
                        tf=tf,  # final time
                        savestep=0.01,
                       )
    df
end
