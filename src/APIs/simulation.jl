"""
Outer construct of `FSimBase.Simulator`.
"""
function Simulator(state0, dyn, p=nothing;
        solver=Tsit5(),
        kwargs...)
    FSimBase.Simulator(state0, dyn, p;
                       solver=solver,
                       kwargs...)
end
