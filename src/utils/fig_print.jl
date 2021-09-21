function fig_print(x_data, y_data, fig_filename, legend_string, ylabel_string; 
					xlabel_string = "t [s]", lw_val = 1.5, N_markers = 10, mks_val = 4, 
					gfs_val = 9, lfs_val = 8, ar_val = :auto, save_file = 1)
		
#------ marker index
marker_idx = zeros(length(x_data))
marker_idx[round.(Int, LinRange(1,length(x_data), N_markers))] = mks_val*ones(N_markers)
# marker_idx = zeros(length(fig_handle.series_list[1].plotattributes[:x]))
# marker_idx[round.(Int, LinRange(1,length(fig_handle.series_list[1].plotattributes[:x]), N_markers))] = mks_val*ones(N_markers)
		
		
#------ figure generation
fig_handle = plot(x_data, y_data, linewidth = lw_val, 
			label = legend_string, legend = :best,
			markersize = marker_idx, markershape = :circle)
		
#------ axes
plot!(fig_handle, guidefontsize = gfs_val, xlabel = xlabel_string)
if ~isnothing(ylabel_string)
	plot!(fig_handle, ylabel = ylabel_string)
end
plot!(fig_handle, aspect_ratio = ar_val)
		
#------ legend
plot!(fig_handle, legendfontsize = lfs_val)
# if ~isnothing(legend_string)
# 	for i in 1:length(fig_handle.series_list)
# 		fig_handle.series_list[i].plotattributes[:label] = legend_string[i]
# 	end
# end

#------ save
if save_file == 1
	fig_dir = "Figures"
	mkpath(fig_dir)
	savefig(fig_handle, joinpath(fig_dir, string("Fig_", fig_filename, ".pdf")))
end

return fig_handle
end

