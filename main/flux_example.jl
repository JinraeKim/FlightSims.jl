using FlightSims
const FS = FlightSims
using Flux
using Flux.Data: DataLoader
using Random
using Transducers


function f(K)
    return function (x)
        K*x
    end
end

function sample_x(n)
    2*rand(n) .- 1.0
end

function main(; seed=1)
    Random.seed!(seed)
    N = 1000  # no. of data
    n = 2
    m = 1
    K = rand(m, n)
    # data generation
    xs = 1:N |> Map(i -> sample_x(n)) |> collect
    fs = xs |> Map(f(K)) |> collect
    partition_ratio = 0.8
    data_train, data_test = FS.partitionTrainTest(zip(xs, fs) |> collect, partition_ratio)  # make a collection of datum and split it; NEVER split data for each labels, e.g., xs_train, xs_test = partitionTrainTest(xs)
    xs_train = data_train |> Map(datum -> datum[1]) |> collect
    fs_train = data_train |> Map(datum -> datum[2]) |> collect
    xs_test = data_test |> Map(datum -> datum[1]) |> collect
    fs_test = data_test |> Map(datum -> datum[2]) |> collect
    # training
    f̂ = Chain(Dense(n, m))  # approximator
    dataloader = DataLoader((hcat(xs_train...), hcat(fs_train...)),  # `hcat` for Flux.jl
                            batchsize=16, shuffle=true)
    loss(x, f) = Flux.Losses.mse(f̂(x), f)
    epochs = 30
    opt = ADAM(1e-3)  # optimiser
    for epoch in 0:epochs
        println("epoch: $(epoch) / $(epochs)")
        if epoch != 0
            # for manual update
            ps = params(f̂)
            for (x, f) in dataloader
                gs = gradient(ps) do
                    loss(x, f)
                end
                Flux.update!(opt, ps, gs)
            end
            # # for automatic update
            # Flux.Optimise.train!(loss, params(f̂), dataloader, opt)
        end
        @show loss(hcat(xs_test...), hcat(fs_test...))  # `hcat` for Flux.jl
    end
end
