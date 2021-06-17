using FlightSims
const FS = FlightSims


FS.@LOG function foo(dx, x, p, t)
    @log p
    @log x_log = x
    @log a, b = p, x
    @log_only k = t^2  # activated only when logged
    return x
end

struct Integrator
    p
end

function test()
    x = [1, 2]
    dx = zero.(x)
    p = [-1]
    t = 0.0
    integrator = Integrator(p)
    return_raw = foo(dx, x, p, t)
    @show return_raw
    returndict = foo__LOG__(x, t, integrator)
    @show returndict
    nothing
end
