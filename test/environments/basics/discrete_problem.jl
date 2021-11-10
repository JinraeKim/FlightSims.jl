using DifferentialEquations


function main()
    t0, tf = 0, 10
    x0 = [1.0, 2, 3]
    """
    dx: next x
    """
    @Loggable function dynamics!(dx, x, p, t; u)
        @log x
        dx .= 0.99*x + u
        @onlylog u_next = dx
    end
    simulator = Simulator(x0, apply_inputs(dynamics!; u=zeros(3));
                          Problem=:Discrete,
                          tf=tf,  # default step length = 1 for Discrete problem
                         )
    df = solve(simulator)
end
