# split data into train and test
"""
    partitionTrainTest(data, at)
Split a dataset into train and test datasets (array).
"""
function partitionTrainTest(data, at=0.8)
    if size(data) |> length != 1
        error("Invalid data type")
    end
    n = length(data)
    idx = Random.shuffle(1:n)
    train_idx = view(idx, 1:floor(Int, at*n))
    test_idx = view(idx, (floor(Int, at*n)+1):n)
    data[train_idx], data[test_idx]
end

