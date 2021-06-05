"""
Supervised-learning-based imitation learning algorithm of deterministic policies.
"""
struct BehaviouralCloning
    π̂
    dataloader::DataLoader
    opt
    function BehaviouralCloning(π̂, states, actions, opt=ADAM(1e-3);
            batchsize=64, shuffle=true
        )
        dataloader = DataLoader((hcat(states...), hcat(actions...)),  # compatible with Flux
                                batchsize=batchsize, shuffle=shuffle)
        new(π̂, dataloader, opt)
    end
end


function (bc::BehaviouralCloning)(s)
    bc.π̂(s)
end


"""
s: states (minibatch) ∈ R^nxN
a: actions (minibatch) ∈ R^mxN
# Examples
for batch in bc.dataloader
    FlightSims.update!(bc, batch...)
end
"""
function update!(bc::BehaviouralCloning, loss, s, a)
    ps = params(bc.π̂)
    gs = gradient(ps) do
        loss(s, a)
    end
    Flux.update!(bc.opt, ps, gs)
end
