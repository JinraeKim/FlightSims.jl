"""
# Notes
- Currently, only iip (isinplace) method is supported.
- Default solver: Tsit5() in OrdinaryDiffEq.jl
"""
function sim(state0, dyn, p=nothing;
        solver=Tsit5(),
        kwargs...
    )
    FSimBase.sim(state0, dyn, p;
                 solver=solver,
                 kwargs...,
                )
end
