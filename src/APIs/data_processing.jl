# # save and load
# """
#     save(path::String,
#     env::AbstractEnv, prob::DiffEqBase.DEProblem, sol::DESolution;
#     process=nothing)
# """
# function FileIO.save(path::String,
#         env::AbstractEnv, prob::DiffEqBase.DEProblem, sol::DESolution;
#         process=nothing,)
#     will_be_saved = Dict("env" => env, "prob" => prob, "sol" => sol)
#     if process != nothing
#         will_be_saved["process"] = process
#     end
#     FileIO.save(path, will_be_saved)
# end

# """
#     load(path::String; with_process=false)
# """
# function JLD2.load(path::String; with_process=false)
#     strs = [:env, :prob, :sol]
#     if with_process
#         push!(strs, :process)
#     end
#     saved_data = FileIO.load(path, String.(strs)...)
#     (; (zip(strs, saved_data) |> Dict)...)  # NamedTuple
# end
