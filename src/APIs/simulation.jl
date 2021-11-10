"""
Outer construct of `FSimBase.Simulator`.
"""
function Simulator(state0, dyn, p=nothing;
        Problem=:ODE,
        solver=nothing,
        kwargs...)
    if Problem == :ODE && solver == nothing
        solver = Tsit5()
    elseif Problem == :Discrete && solver == nothing
        solver = FunctionMap()
    end
    FSimBase.Simulator(state0, dyn, p;
                       Problem=Problem,
                       solver=solver,
                       kwargs...)
end
